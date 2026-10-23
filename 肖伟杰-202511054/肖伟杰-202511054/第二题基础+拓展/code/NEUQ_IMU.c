/*********************************************************************************************************************
* 文件名称          NEUQ_IMU
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第二题：六轴采集 + 六轴独立滤波 + 零偏校准 实现
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "NEUQ_FilterCoef.h"    /* 低通系数公式（固件和主机端验证共用同一份） */
#include "NEUQ_Attitude.h"
#include "NEUQ_IMU.h"

volatile float imu_raw[AXIS_NUM];
volatile float imu_filt[AXIS_NUM];
volatile float imu_mg[3];
volatile float imu_cdps[3];
volatile float imu_bias[3];

static float   s_alpha = 0.0f;              /* 低通系数 */
static uint8_t s_online = 0;
static uint8_t s_calib  = 0;                /* 1 = 正在校准 */
static uint16_t s_cnt   = 0;                /* 校准已累计样本 */
static float   s_sum[3];                    /* 校准累加 */

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     设置截止频率：立刻重算 alpha，六个滤波器一起变
* 输入参数     fc_x10 : 0.1Hz 为单位（100 = 10.0 Hz）
* 备注信息     公式本身在 NEUQ_FilterCoef.h 里，固件和验证程序共用一份，不会发散。
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_IMU_SetFc(uint16_t fc_x10)
{
    float fc = (float)fc_x10 / 10.0f;
    float dt = (float)IMU_PERIOD_MS / 1000.0f;      /* 5 ms → 0.005 s */

    if (fc < 1.0f)   { fc = 1.0f; }
    if (fc > 100.0f) { fc = 100.0f; }

    s_alpha = NEUQ_FilterCoef(fc, dt);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     初始化：MPU6050 + 默认滤波系数
*------------------------------------------------------------------------------------------------------------------*/
uint8_t NEUQ_IMU_Init(void)
{
    uint8_t i;

    for (i = 0; i < AXIS_NUM; i++)
    {
        imu_raw[i]  = 0.0f;
        imu_filt[i] = 0.0f;
    }
    for (i = 0; i < 3U; i++)
    {
        imu_mg[i]   = 0.0f;
        imu_cdps[i] = 0.0f;
        imu_bias[i] = 0.0f;
    }

    /* ★ 这里**不能**无条件 SetFc：截止频率由菜单/Flash 决定（NEUQ_Menu2_Init 已经设过一次），
     *   以前这里写死 SetFc(100)，导致"断电重启后屏幕显示恢复了、滤波器实际还是 10 Hz"。
     *   只有从没设过（s_alpha 还是 0）时才给个兜底值。 */
    if (s_alpha <= 0.0f)
    {
        NEUQ_IMU_SetFc(100);        /* 兜底 10.0 Hz */
    }

    s_online = NEUQ_MPU6050_init();

    return s_online;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     启动一次静态零偏校准（上电时、或菜单里点 Gyro Calib）
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_IMU_Calib(void)
{
    s_cnt      = 0;
    s_sum[0]   = 0.0f;
    s_sum[1]   = 0.0f;
    s_sum[2]   = 0.0f;
    s_calib    = 1;
    NEUQ_Attitude_Reset();          /* 顺便把当前姿态记为 0° */
}

uint8_t NEUQ_IMU_CalibBusy(void)
{
    return s_calib;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     传感器是否在线（= NEUQ_MPU6050_init() 的结果）
* 备注信息     六轴数据全 0 的唯一原因就是它等于 0：NEUQ_IMU_Tick() 会直接 return。
*------------------------------------------------------------------------------------------------------------------*/
uint8_t NEUQ_IMU_Online(void)
{
    return s_online;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     5 ms 中断节拍
* 备注信息     一次读 14 字节，所以六个轴来自同一次采样；不做刷屏和串口。
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_IMU_Tick(void)
{
    uint8_t i;
    int16_t raw;

    if (s_online == 0U)
    {
        return;
    }

    NEUQ_MPU6050_read_data();

    /* ① 原始计数 + 六个轴各自滤波 */
    raw = mpu6050_acc_x;
    imu_raw[AXIS_ACC_X]  = (float)raw;
    raw = mpu6050_acc_y;
    imu_raw[AXIS_ACC_Y]  = (float)raw;
    raw = mpu6050_acc_z;
    imu_raw[AXIS_ACC_Z]  = (float)raw;
    raw = mpu6050_gyro_x;
    imu_raw[AXIS_GYR_X]  = (float)raw;
    raw = mpu6050_gyro_y;
    imu_raw[AXIS_GYR_Y]  = (float)raw;
    raw = mpu6050_gyro_z;
    imu_raw[AXIS_GYR_Z]  = (float)raw;
    
    //滤波

    for (i = 0; i < AXIS_NUM; i++)
    {
        imu_filt[i] += s_alpha * (imu_raw[i] - imu_filt[i]);
    }

    /* ② 换算成显示单位：加速度 mg，角速度 0.01 dps
     *    ±8g   → 4096 LSB/g  → 1 LSB = 0.244 mg
     *    ±2000dps → 16.4 LSB/(°/s) → 1 LSB = 6.1 (0.01dps)
     *    显示也取滤波后的值：调截止频率时六轴页能直接看到抖动变小/变大。 */
    imu_mg[0] = imu_filt[AXIS_ACC_X] * 0.2441f;
    imu_mg[1] = imu_filt[AXIS_ACC_Y] * 0.2441f;
    imu_mg[2] = imu_filt[AXIS_ACC_Z] * 0.2441f;

    {
        float cdps_per_count = MPU6050_gyro_transition(1) * 100.0f;   /* 计数 → 0.01dps */

        imu_cdps[0] = imu_filt[AXIS_GYR_X] * cdps_per_count - imu_bias[0];
        imu_cdps[1] = imu_filt[AXIS_GYR_Y] * cdps_per_count - imu_bias[1];
        imu_cdps[2] = imu_filt[AXIS_GYR_Z] * cdps_per_count - imu_bias[2];
    }

    /* ③ 零偏校准：累计 1.5 s 求平均 */
    if (s_calib != 0U)
    {
        s_sum[0] += MPU6050_gyro_transition(mpu6050_gyro_x) * 100.0f;
        s_sum[1] += MPU6050_gyro_transition(mpu6050_gyro_y) * 100.0f;
        s_sum[2] += MPU6050_gyro_transition(mpu6050_gyro_z) * 100.0f;
        s_cnt++;

        if (s_cnt >= IMU_CALIB_CNT)
        {
            imu_bias[0] = s_sum[0] / (float)IMU_CALIB_CNT;
            imu_bias[1] = s_sum[1] / (float)IMU_CALIB_CNT;
            imu_bias[2] = s_sum[2] / (float)IMU_CALIB_CNT;
            s_calib = 0;
            NEUQ_Attitude_Reset();
        }
    }

    /* ④ 姿态解算：用原始加速度 + 扣掉零偏的角速度，切页面照常跑 */
    NEUQ_Attitude_Update(MPU6050_gyro_transition(mpu6050_gyro_x) - imu_bias[0] * 0.01f,
                         MPU6050_gyro_transition(mpu6050_gyro_y) - imu_bias[1] * 0.01f,
                         MPU6050_gyro_transition(mpu6050_gyro_z) - imu_bias[2] * 0.01f,
                         MPU6050_acc_transition(mpu6050_acc_x),
                         MPU6050_acc_transition(mpu6050_acc_y),
                         MPU6050_acc_transition(mpu6050_acc_z));
}
