/********************************************************************************
 * 文件    : Task_Gyro.c
 * 功能    : 陀螺仪六轴数据采集 + 低通滤波 + VOFA+ 波形上传 + 参数存 Flash
 * 说明    : 1. 本文件放在 CODE 文件夹下，EIDE 工程会自动编译(Keil 需手动添加)
 *           2. 菜单/按键/绘制相关代码已拆分到 Key.c
 *           3. 代码里所有【需修改】标注，都是需要按你的硬件/接线调整的地方
 ********************************************************************************/
#include "NEUQ_lib_include.h"
#include "Task_Gyro.h"
#include "Key.h"
#include "Attitude.h"

/*=========================== 【需修改】区：硬件相关配置 ===========================*/
// VOFA+ 波形上传串口（默认用调试串口 UART_0 = PA9/PA10，波特率115200）
#define VOFA_UART     UART_0
#define VOFA_BAUD     UART_BAUD_115200

// 采样周期(ms)：10ms = 100Hz
#define SAMPLE_MS     10

// 参数保存的 Flash 扇区编号（W25Q16 最后一个扇区，一般不用改）
#define CFG_SECTOR    NEUQ_FLASH_LAST_SECTOR_ID
/*=========================== 【需修改】区结束 ===========================*/

#define CFG_MAGIC  0x5A5AA5A5UL

/*=============================== 全局变量 ===============================*/
FilterConfig_t cfg;                  // 当前配置
float raw[AXIS_NUM];                 // 六轴原始值（物理单位 g / deg/s）
static float filt[AXIS_NUM];         // 六轴滤波后值
const char Axis_Names[AXIS_NUM][3] = {"Ax","Ay","Az","Gx","Gy","Gz"};
static uint32_t last_sample = 0;     // 上次采样时间

/*=============================== Flash 存取 ===============================*/

// 保存配置到 Flash（Key.c 的菜单确认时会调用）
void Cfg_Save(void){
    uint8_t buf[sizeof(FilterConfig_t)];
    cfg.magic = CFG_MAGIC;
    memcpy(buf, &cfg, sizeof(FilterConfig_t));
    NEUQ_Flash_write_sector(CFG_SECTOR, buf, sizeof(FilterConfig_t));  // 会先擦除整个扇区
}

static void Cfg_Load(void){
    // 默认值
    cfg.magic    = CFG_MAGIC;
    cfg.alpha    = 0.10f;
    cfg.sel_axis = AXIS_GX;
    cfg.send_on  = 0;

    FilterConfig_t tmp;
    NEUQ_Flash_read_sector(CFG_SECTOR, (uint8_t *)&tmp, sizeof(FilterConfig_t));
    if(tmp.magic == CFG_MAGIC){                       // 校验通过则用 Flash 里的值，并做范围保护
        if(tmp.alpha >= 0.01f && tmp.alpha <= 1.00f) cfg.alpha = tmp.alpha;
        if(tmp.sel_axis < AXIS_NUM)                  cfg.sel_axis = tmp.sel_axis;
        cfg.send_on = (tmp.send_on != 0) ? 1 : 0;
    }
}

/*=============================== 采集与滤波 ===============================*/

// 采样 + 一阶低通滤波（六个轴独立滤波）
static void Sample_And_Filter(void){
    NEUQ_MPU6050_read_data();

    float r[AXIS_NUM];
    r[AXIS_AX] = MPU6050_acc_transition(mpu6050_acc_x);    // g
    r[AXIS_AY] = MPU6050_acc_transition(mpu6050_acc_y);
    r[AXIS_AZ] = MPU6050_acc_transition(mpu6050_acc_z);
    r[AXIS_GX] = MPU6050_gyro_transition(mpu6050_gyro_x);  // deg/s
    r[AXIS_GY] = MPU6050_gyro_transition(mpu6050_gyro_y);
    r[AXIS_GZ] = MPU6050_gyro_transition(mpu6050_gyro_z);

    for(uint8_t i = 0; i < AXIS_NUM; i++){
        raw[i]  = r[i];
        filt[i] = filt[i] + cfg.alpha * (r[i] - filt[i]);  // 低通滤波公式
    }
}

/*=============================== 姿态解算（四元数） ===============================*/

// 静态校准：上电/按键触发时模块须保持静止。
// 采样 N 次取平均，得到陀螺仪零偏(deg/s)；再用平均加速度确定初始横滚/俯仰，
// 航向归 0（六轴无绝对航向参考，以当前方向为相对参考）。
// 注：上电后先预热、丢弃前几十个不稳定的读数再取平均，零偏测得越准 yaw 漂移越小。
#define CALIB_SAMPLES   1000
void Task_Gyro_Calibrate(void){
    // 预热 + 丢弃传感器上电后的初始不稳定读数
    NEUQ_Delay_ms(300);
    for(int i = 0; i < 50; i++){
        NEUQ_MPU6050_read_data();
        NEUQ_Delay_ms(2);
    }

    double gx = 0, gy = 0, gz = 0;          // 陀螺仪零偏累加
    double ax = 0, ay = 0, az = 0;          // 加速度平均（用于初始姿态）

    for(int i = 0; i < CALIB_SAMPLES; i++){
        NEUQ_MPU6050_read_data();
        gx += MPU6050_gyro_transition(mpu6050_gyro_x);
        gy += MPU6050_gyro_transition(mpu6050_gyro_y);
        gz += MPU6050_gyro_transition(mpu6050_gyro_z);
        ax += MPU6050_acc_transition(mpu6050_acc_x);
        ay += MPU6050_acc_transition(mpu6050_acc_y);
        az += MPU6050_acc_transition(mpu6050_acc_z);
        NEUQ_Delay_ms(2);
    }

    Attitude_SetGyroBias((float)(gx / CALIB_SAMPLES),
                         (float)(gy / CALIB_SAMPLES),
                         (float)(gz / CALIB_SAMPLES));
    Attitude_SetInitFromAccel((float)(ax / CALIB_SAMPLES),
                              (float)(ay / CALIB_SAMPLES),
                              (float)(az / CALIB_SAMPLES));
}

/*=============================== VOFA+ 波形上传 ===============================*/

// VOFA+ JustFloat 协议：每个通道 4 字节 float（小端），一帧结束加 00 00 80 7F。
// 这里一帧发 2 个通道：原始值 + 滤波值（同一单位、同一尺度，便于成对对比）。
static void VOFA_Send(void){
    float a = raw[cfg.sel_axis];
    float b = filt[cfg.sel_axis];
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    NEUQ_Uart_write_data(VOFA_UART, (uint8_t *)&a, 4);
    NEUQ_Uart_write_data(VOFA_UART, (uint8_t *)&b, 4);
    NEUQ_Uart_write_data(VOFA_UART, tail, 4);
}

/*=============================== 初始化 ===============================*/

void Task_Gyro_Init(void){
    // 1. 串口（VOFA+ 用；UART_0 已由 NEUQ_Debug_init 初始化，这里再按需配置一次）
    NEUQ_Uart_init(VOFA_UART, VOFA_BAUD);

    // 2. 按键 + 屏幕（在 Key.c 里）
    Key_Init();

    // 3. Flash + 读取保存的配置
    NEUQ_Flash_init();
    Cfg_Load();

    // 4. MPU6050
    NEUQ_MPU6050_init();

    // 5. 滤波初值清零
    for(uint8_t i = 0; i < AXIS_NUM; i++){ raw[i] = 0; filt[i] = 0; }

    // 6. 姿态解算初始化 + 静态零偏校准（此时模块必须保持静止！）
    Attitude_Init();
    Task_Gyro_Calibrate();

    last_sample = HAL_GetTick();
}

/*=============================== 主循环 ===============================*/

void Task_Gyro_Loop(void){
    // 采样 + 滤波 + 姿态解算 + 上传。
    // 采样与姿态解算始终进行（切换页面也不中断），波形上传仅在"六轴数据显示"页且开启发送时进行。
    uint32_t now = HAL_GetTick();
    if(now - last_sample >= SAMPLE_MS){
        float dt = (now - last_sample) * 0.001f;   // 实际步长(s)，比固定 SAMPLE_MS 更稳
        last_sample = now;

        Sample_And_Filter();

        // 四元数姿态更新（原始角速度 deg/s + 原始加速度 g）
        Attitude_Update(raw[AXIS_GX], raw[AXIS_GY], raw[AXIS_GZ],
                        raw[AXIS_AX], raw[AXIS_AY], raw[AXIS_AZ], dt);

        if(cfg.send_on && page == PAGE_DISPLAY){
            VOFA_Send();        // 发送当前选中轴的 原始+滤波 波形
        }
    }
}
