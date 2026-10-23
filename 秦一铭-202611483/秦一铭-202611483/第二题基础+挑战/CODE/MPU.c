#include "NEUQ_lib_include.h"

float raw[6];
float filt[6];
int32_t param[6];

float roll, pitch, yaw;

static float bias[3];               // 陀螺零偏，deg/s
static float q[4] = {1, 0, 0, 0};   // 四元数 w,x,y,z

#define DT      0.002f      // 中断周期 2ms
#define R2D     57.29578f   // 弧度转角度
#define D2R     0.01745329f // 角度转弧度
#define KP      1.0f        // 加速度计修正强度
#define KI      0.002f      // 零偏修正强度
#define A_DEF   20          // 默认 α=0.20

// 读一次 MPU6050，转成物理量
static void MPU_Read(void){
    NEUQ_MPU6050_read_data();

    raw[0] = MPU6050_gyro_transition(mpu6050_gyro_x);
    raw[1] = MPU6050_gyro_transition(mpu6050_gyro_y);
    raw[2] = MPU6050_gyro_transition(mpu6050_gyro_z);
    raw[3] = MPU6050_acc_transition(mpu6050_acc_x);
    raw[4] = MPU6050_acc_transition(mpu6050_acc_y);
    raw[5] = MPU6050_acc_transition(mpu6050_acc_z);
}

// 一阶低通：α 越大越跟手，但噪声大；越小越平滑，但滞后大
static void MPU_Filter(void){
    uint8_t i;

    for (i = 0; i < 6; i++)
        filt[i] += (float)param[i] * 0.01f * (raw[i] - filt[i]);//新滤波值 = 旧滤波值 + α × (本次原始值 - 旧滤波值)
}

// 归一化，四元数长度必须保持为 1
static void Quat_Norm(void){
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
}

// 四元数转欧拉角:roll 绕 X 轴，pitch 绕 Y 轴，yaw 绕 Z 轴,单位是度
static void Quat_Angle(void){
    roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]),
                   1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2])) * R2D;
    pitch = asinf(2.0f*(q[0]*q[2] - q[3]*q[1])) * R2D;
    yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]),
                   1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3])) * R2D;
}

/* Mahony 互补滤波：陀螺积分提供短期快速变化，加速度计提供长期重力方向，两者互相修正
 * ① 角速度积分，让四元数长出新姿态
 * ② 加速度计量到的重力方向，和四元数推算的重力方向比一下，
 *    差多少就回头修正角速度
 * ③ 归一化后转成欧拉角
 */
static void Attitude_Update(void){
    float gx, gy, gz;           // 去零偏、换成 rad/s 的角速度
    float ax, ay, az, n;        // 加速度及其模
    float vx, vy, vz;           // 四元数推算的重力方向
    float ex, ey, ez;           // 两个重力方向的误差
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float h = DT * 0.5f;

    //第一步：陀螺去零偏，转弧度
    gx = (filt[0] - bias[0]) * D2R;
    gy = (filt[1] - bias[1]) * D2R;
    gz = (filt[2] - bias[2]) * D2R;

    ax = filt[3];
    ay = filt[4];
    az = filt[5];
    n  = sqrtf(ax*ax + ay*ay + az*az);
    ax /= n;
    ay /= n;
    az /= n;

    //第二步：用四元数推算“重力应该指向哪”
    vx = 2.0f * (q1*q3 - q0*q2);
    vy = 2.0f * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    //第三步：实测重力方向和推算方向做叉乘，得到误差
    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

    // 用误差修正角速度，并慢慢修正零偏
    gx += KP * ex;
    gy += KP * ey;
    gz += KP * ez;
    bias[0] -= KI * ex * R2D * DT;
    bias[1] -= KI * ey * R2D * DT;
    bias[2] -= KI * ez * R2D * DT;

    // 一阶龙格库塔积分
    q[0] += (-q1*gx - q2*gy - q3*gz) * h;
    q[1] += ( q0*gx + q2*gz - q3*gy) * h;
    q[2] += ( q0*gy - q1*gz + q3*gx) * h;
    q[3] += ( q0*gz + q1*gy - q2*gx) * h;

    Quat_Norm();  // 归一化，防止数值积分让长度跑偏
    Quat_Angle(); //// 四元数转欧拉角
}

// 上电校准：静止 2 秒，陀螺平均值当零偏
static void MPU_Cal(void){
    float sx = 0, sy = 0, sz = 0;
    float r, p;
    uint16_t i;

    NEUQ_Delay_ms(200);                 // 等模块稳定

    for (i = 0; i < 1000; i++){
        MPU_Read();
        MPU_Filter();
        sx += raw[0];
        sy += raw[1];
        sz += raw[2];
        NEUQ_Delay_ms(2);               // 1000 × 2ms = 2s
    }

    bias[0] = sx / 1000.0f;
    bias[1] = sy / 1000.0f;
    bias[2] = sz / 1000.0f;             //静止采 1000 次陀螺，求平均

    // 用加速度算初始姿态
    roll  =  atan2f(filt[4], filt[5]) * R2D;
    pitch = -atan2f(filt[3], sqrtf(filt[4]*filt[4] + filt[5]*filt[5])) * R2D;
    yaw   = 0;

    r = roll * D2R * 0.5f;
    p = pitch * D2R * 0.5f;
    q[0] = cosf(r) * cosf(p);
    q[1] = sinf(r) * cosf(p);
    q[2] = cosf(r) * sinf(p);
    q[3] = sinf(r) * sinf(p);
    Quat_Norm();
}

// 初始化：给默认参数，校准零偏
void MPU_Init(void){
    uint8_t i;

    for (i = 0; i < 6; i++) param[i] = A_DEF;

    NEUQ_MPU6050_init();
    MPU_Cal();
}

// 中断里每 2ms 调一次：读取 → 滤波 → 姿态解算
void MPU_Update(void){
    MPU_Read();
    MPU_Filter();
    Attitude_Update();
}

/* ============ Flash 存读参数 ============*/
//擦除后整个扇区所有位都回到 1（没有电子），一个字节就是 0xFF，四个字节按 int32_t 读出来就是 0xFFFFFFFF，不在 1~99 之内，所以自动用默认值。
void Param_Load(void){
    int32_t buf[6];
    uint8_t i;

    NEUQ_Flash_read_sector(NEUQ_FLASH_LAST_SECTOR_ID, (uint8_t *)buf, sizeof(buf));

    for (i = 0; i < 6; i++)
        param[i] = (buf[i] >= 1 && buf[i] <= 99) ? buf[i] : A_DEF;
}

void Param_Save(void){
    NEUQ_Flash_write_sector(NEUQ_FLASH_LAST_SECTOR_ID, (uint8_t *)param, sizeof(param));
}