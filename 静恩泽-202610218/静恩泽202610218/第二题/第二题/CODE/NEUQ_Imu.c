#include "NEUQ_lib_include.h"
#include "NEUQ_Imu.h"
#include <math.h>

/*
 * 本模块以 100Hz（10ms 中断节拍）持续采集 MPU6050 六轴数据并做一阶低通滤波，
 * 同时用四元数（Mahony 互补滤波）解算姿态角，并可按需通过 VOFA+ 的 FireWater
 * 文本协议把“某单轴原始值 + 滤波值”成对发送到上位机。
 *
 * 坐标约定：模块水平静置、Z 轴朝上，X/Y 为 MPU6050 芯片默认方向；
 *           Roll 绕 X 轴、Pitch 绕 Y 轴、Yaw 绕 Z 轴，Yaw 以启动初始方向为相对 0。
 */

IMU_Config imu_cfg;

static float raw[IMU_AXIS_COUNT];    // 原始物理量：加速度(g)、角速度(°/s)
static float filt[IMU_AXIS_COUNT];   // 低通滤波结果
static float gyro_bias[3];           // 陀螺仪零偏(°/s)

// 四元数与欧拉角
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

volatile uint8_t imu_tick_flag = 0;

// Mahony 互补滤波参数（无磁力计，仅修正 Roll/Pitch 漂移，Yaw 为相对参考）
#define IMU_MAHONY_KP   2.0f
#define IMU_MAHONY_KI   0.0f
static float integral_fbx = 0.0f, integral_fby = 0.0f, integral_fbz = 0.0f;

#define DEG2RAD (0.017453292519943295f)
#define RAD2DEG (57.29577951308232f)

// Flash 配置存储：使用倒数第 2 个扇区，避开演示程序可能占用的最后扇区
#define IMU_CFG_SECTOR    510
#define IMU_CFG_MAGIC     0x494D5531UL   // "IMU1"

typedef struct {
    uint32_t magic;
    int32_t  filter_k;
    int32_t  send_axis;
} ImuFlashCfg;

static const char *axis_name[IMU_AXIS_COUNT] = {
    "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ"
};

//==============================================================================================
// 内部：把 MPU6050 全局原始计数转换为物理量
//==============================================================================================
static void Imu_UpdateRaw(void){
    raw[IMU_AXIS_ACC_X]  = MPU6050_acc_transition(mpu6050_acc_x);
    raw[IMU_AXIS_ACC_Y]  = MPU6050_acc_transition(mpu6050_acc_y);
    raw[IMU_AXIS_ACC_Z]  = MPU6050_acc_transition(mpu6050_acc_z);
    raw[IMU_AXIS_GYRO_X] = MPU6050_gyro_transition(mpu6050_gyro_x);
    raw[IMU_AXIS_GYRO_Y] = MPU6050_gyro_transition(mpu6050_gyro_y);
    raw[IMU_AXIS_GYRO_Z] = MPU6050_gyro_transition(mpu6050_gyro_z);
}

//==============================================================================================
// 内部：静态陀螺仪零偏校准（静止时多次采样取平均）
//==============================================================================================
static void Imu_CalibrateGyro(void){
    const int32_t N = 400;
    float sum[3] = {0.0f, 0.0f, 0.0f};
    for(int32_t i = 0; i < N; i++){
        NEUQ_MPU6050_read_data();
        sum[0] += (float)mpu6050_gyro_x;
        sum[1] += (float)mpu6050_gyro_y;
        sum[2] += (float)mpu6050_gyro_z;
        NEUQ_Delay_ms(1);
    }
    gyro_bias[0] = MPU6050_gyro_transition((int16_t)(sum[0] / N));
    gyro_bias[1] = MPU6050_gyro_transition((int16_t)(sum[1] / N));
    gyro_bias[2] = MPU6050_gyro_transition((int16_t)(sum[2] / N));

    NEUQ_MPU6050_read_data();
    Imu_UpdateRaw();
}

//==============================================================================================
// 内部：从 Flash 加载有效配置，无效记录则保留默认值
//==============================================================================================
static void Imu_LoadConfig(void){
    uint8_t buf[sizeof(ImuFlashCfg)];
    ImuFlashCfg *c = (ImuFlashCfg *)buf;
    if(NEUQ_Flash_read_sector(IMU_CFG_SECTOR, buf, sizeof(ImuFlashCfg)) == NEUQ_FLASH_OK){
        if(c->magic == IMU_CFG_MAGIC){
            if(c->filter_k >= 0 && c->filter_k <= 900){
                imu_cfg.filter_k = c->filter_k;
            }
            if(c->send_axis >= 0 && c->send_axis < IMU_AXIS_COUNT){
                imu_cfg.send_axis = c->send_axis;
            }
        }
    }
}

//==============================================================================================
// 内部：Mahony 互补滤波 + 四元数姿态更新，dt = 10ms
//==============================================================================================
static void Imu_AttitudeUpdate(void){
    const float dt = 0.01f;   // 100Hz

    // 1. 加速度归一化（单位重力方向）
    float ax = raw[IMU_AXIS_ACC_X];
    float ay = raw[IMU_AXIS_ACC_Y];
    float az = raw[IMU_AXIS_ACC_Z];
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm < 1e-6f) norm = 1e-6f;
    ax /= norm; ay /= norm; az /= norm;

    // 2. 角速度（rad/s），扣除零偏
    float gx = (raw[IMU_AXIS_GYRO_X] - gyro_bias[0]) * DEG2RAD;
    float gy = (raw[IMU_AXIS_GYRO_Y] - gyro_bias[1]) * DEG2RAD;
    float gz = (raw[IMU_AXIS_GYRO_Z] - gyro_bias[2]) * DEG2RAD;

    // 3. 由四元数估计的重力方向
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 4. 误差 = 实测加速度 叉乘 估计重力
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    // 5. PI 校正
    if(IMU_MAHONY_KI > 0.0f){
        integral_fbx += IMU_MAHONY_KI * ex * dt;
        integral_fby += IMU_MAHONY_KI * ey * dt;
        integral_fbz += IMU_MAHONY_KI * ez * dt;
        gx += integral_fbx; gy += integral_fby; gz += integral_fbz;
    } else {
        integral_fbx = integral_fby = integral_fbz = 0.0f;
    }
    gx += IMU_MAHONY_KP * ex;
    gy += IMU_MAHONY_KP * ey;
    gz += IMU_MAHONY_KP * ez;

    // 6. 四元数一阶积分
    float halfT = dt * 0.5f;
    float q0n = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
    float q1n = q1 + ( q0*gx + q2*gz - q3*gy) * halfT;
    float q2n = q2 + ( q0*gy - q1*gz + q3*gx) * halfT;
    float q3n = q3 + ( q0*gz + q1*gy - q2*gx) * halfT;

    // 7. 四元数归一化
    norm = sqrtf(q0n*q0n + q1n*q1n + q2n*q2n + q3n*q3n);
    if(norm < 1e-6f) norm = 1e-6f;
    q0 = q0n / norm; q1 = q1n / norm; q2 = q2n / norm; q3 = q3n / norm;

    // 8. 四元数 -> 欧拉角（°）
    roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;
    pitch = asinf(2.0f*(q0*q2 - q3*q1)) * RAD2DEG;
    yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;
}

//==============================================================================================
// 内部：float -> 十进制字符串（用整数运算实现，规避 microlib 不支持 %f 格式化的问题）
//       buf 输出缓冲区，value 待转换值，decimals 小数点后位数
//       返回写入字符数（不含结尾 '\0'）
//==============================================================================================
static int32_t Imu_Ftoa(char *buf, float value, int32_t decimals){
    char *p = buf;

    // 分离符号
    if(value < 0.0f){
        *p++ = '-';
        value = -value;
    }

    // 四舍五入到 decimals 位：放大 10^decimals 后取整
    float   scale_f = 1.0f;
    int32_t scale   = 1;
    for(int32_t i = 0; i < decimals; i++){
        scale_f *= 10.0f;
        scale   *= 10;
    }
    int32_t scaled = (int32_t)(value * scale_f + 0.5f);

    // 四舍五入后为 0 时去掉负号，避免输出 "-0.000"
    if(scaled == 0 && p > buf && *(p - 1) == '-'){
        p--;
    }

    int32_t int_part  = scaled / scale;
    int32_t frac_part = scaled % scale;

    // 整数部分
    char tmp[16];
    int32_t n = 0;
    if(int_part == 0){
        tmp[n++] = '0';
    }else{
        while(int_part > 0){
            tmp[n++] = (char)('0' + (int_part % 10));
            int_part /= 10;
        }
        for(int32_t i = 0; i < n / 2; i++){
            char c = tmp[i];
            tmp[i] = tmp[n - 1 - i];
            tmp[n - 1 - i] = c;
        }
    }
    for(int32_t i = 0; i < n; i++){
        *p++ = tmp[i];
    }

    // 小数部分（固定 decimals 位，自动补前导 0）
    if(decimals > 0){
        *p++ = '.';
        char ftmp[16];
        for(int32_t i = 0; i < decimals; i++){
            ftmp[decimals - 1 - i] = (char)('0' + (frac_part % 10));
            frac_part /= 10;
        }
        for(int32_t i = 0; i < decimals; i++){
            *p++ = ftmp[i];
        }
    }

    *p = '\0';
    return (int32_t)(p - buf);
}

//==============================================================================================
// 内部：VOFA+ FireWater 文本协议发送单轴（原始值 + 滤波值 成对，来自同一次采样）
//       帧格式：<raw>,<filt>\n   两通道同帧，即同一时间基准
//==============================================================================================
static void Imu_SendVofa(void){
    int32_t axis = imu_cfg.send_axis;
    if(axis < 0 || axis >= IMU_AXIS_COUNT) axis = 0;

    char line[64];
    int32_t len = 0;
    len += Imu_Ftoa(line + len, raw[axis],  4);
    line[len++] = ',';
    len += Imu_Ftoa(line + len, filt[axis], 4);
    line[len++] = '\n';
    line[len]   = '\0';

    NEUQ_Uart_write_data(UART_0, (uint8_t *)line, (uint32_t)len);
}

//==============================================================================================
// 对外接口
//==============================================================================================
void NEUQ_Imu_Init(void){
    NEUQ_MPU6050_init();
    NEUQ_Flash_init();

    // 无有效记录时的默认值
    imu_cfg.filter_k    = 500;   // alpha = 0.5
    imu_cfg.send_axis   = IMU_AXIS_ACC_X;
    imu_cfg.send_enable = 0;

    Imu_LoadConfig();
    Imu_CalibrateGyro();

    for(int32_t i = 0; i < IMU_AXIS_COUNT; i++){
        filt[i] = raw[i];
    }

    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    roll = pitch = yaw = 0.0f;
}

void NEUQ_Imu_Tick(void){
    imu_tick_flag = 1;
}

void NEUQ_Imu_Process(void){
    if(!imu_tick_flag) return;
    imu_tick_flag = 0;

    NEUQ_MPU6050_read_data();
    Imu_UpdateRaw();

    // 六轴低通滤波：系数随 filter_k 实时变化，改参数立即见效
    float alpha = (float)imu_cfg.filter_k / 1000.0f;
    for(int32_t i = 0; i < IMU_AXIS_COUNT; i++){
        filt[i] += alpha * (raw[i] - filt[i]);
    }

    Imu_AttitudeUpdate();

    if(imu_cfg.send_enable){
        Imu_SendVofa();
    }
}

float NEUQ_Imu_Raw(int32_t axis){
    if(axis < 0 || axis >= IMU_AXIS_COUNT) return 0.0f;
    return raw[axis];
}

float NEUQ_Imu_Filtered(int32_t axis){
    if(axis < 0 || axis >= IMU_AXIS_COUNT) return 0.0f;
    return filt[axis];
}

float NEUQ_Imu_Roll(void){ return roll; }
float NEUQ_Imu_Pitch(void){ return pitch; }
float NEUQ_Imu_Yaw(void){ return yaw; }

const char *NEUQ_Imu_AxisName(int32_t axis){
    if(axis < 0 || axis >= IMU_AXIS_COUNT) return "?";
    return axis_name[axis];
}

void NEUQ_Imu_SaveConfig(void){
    ImuFlashCfg c;
    c.magic     = IMU_CFG_MAGIC;
    c.filter_k  = imu_cfg.filter_k;
    c.send_axis = imu_cfg.send_axis;
    NEUQ_Flash_write_sector(IMU_CFG_SECTOR, (uint8_t *)&c, sizeof(ImuFlashCfg));
}
