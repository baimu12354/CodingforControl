/*
 * NEUQ_IMU.h
 * MPU6050 六轴连续采集 + 六轴各自独立的数字低通滤波
 *
 * 这个模块跟"当前在哪个页面"没关系，只要单片机在跑，采集和滤波就一直进行。
 */
#ifndef _NEUQ_IMU_H_
#define _NEUQ_IMU_H_

#include "NEUQ_lib_include.h"

// 采样周期，2ms就是500Hz
// I2C读一次14字节约0.45ms，500Hz下占22%左右的CPU，剩下的够刷屏和发串口。
// 觉得CPU紧张可以改大，但姿态更新周期会跟着变长，静态漂移会变大。
#define NEUQ_IMU_PERIOD_MS            (2U)

// 量程和灵敏度
// 注意这两个常数必须和 NEUQ_IMU_Init() 里真正写进MPU6050的量程一致，不然算出来的值全是错的。
// 加速度±4g   -> 8192 LSB/g，分辨率约0.00012g，也不容易被磕碰打饱和
// 角速度±2000 -> 16.4 LSB/(°/s)，手快挥也不会饱和，姿态解算最怕角速度饱和
#define NEUQ_IMU_ACC_LSB_PER_G        (8192.0f)
#define NEUQ_IMU_GYRO_LSB_PER_DPS     (16.4f)

// 加速度可信窗口，单位g
// 静止或慢慢倾斜的时候|a|大约是1g。如果明显偏离1g，说明模块正在被推被敲，
// 这时候加速度计测的不是重力方向，这次就别拿它去修正姿态了。
#define NEUQ_IMU_ACC_TRUST_MIN        (0.75f)
#define NEUQ_IMU_ACC_TRUST_MAX        (1.25f)

// 二阶IIR低通滤波器，巴特沃斯，Q=0.707
// 用的是Direct Form I，每个通道一份状态，六轴互不影响。
// 截止频率可以随时改，改完立刻重算系数，波形上看得到变化。
typedef struct{
    float b0, b1, b2;       // 前馈系数
    float a1, a2;           // 反馈系数，a0已经归一化成1了
    float x1, x2;           // 输入历史
    float y1, y2;           // 输出历史
} NEUQ_Biquad_t;

// 六轴数据实体，这个结构体就是后台真实状态，菜单只是把它显示出来
typedef struct{
    int16_t raw[6];             // [0..2]加速度原始计数 [3..5]角速度原始计数
    float   acc_g_raw[3];       // 未滤波加速度，g
    float   acc_g[3];           // 滤波后加速度，g
    float   gyro_dps_raw[3];    // 未滤波角速度，°/s
    float   gyro_dps[3];        // 滤波后角速度，°/s，姿态解算用的就是这个
    float   gyro_bias_dps[3];   // 静态零偏校准结果，°/s
    uint32_t ok_cnt;            // 成功采样次数
    uint32_t err_cnt;           // 连续读失败次数
    uint8_t  online;            // 1 = 传感器在线
} NEUQ_IMU_Data_t;

extern NEUQ_IMU_Data_t g_imu;

// 初始化MPU6050，返回1成功0没找到
// 库里的NEUQ_MPU6050_init()会把内部DLPF配成44Hz，那样"原始数据"已经被芯片自己磨平了，
// 软件滤波前后几乎看不出区别。所以这里初始化完之后重新写寄存器，
// 把内部DLPF放宽到188Hz，让原始数据保留真实抖动，滤波效果才看得出来。
uint8_t NEUQ_IMU_Init(void);

// 读一次传感器，换算物理量，六轴分别做低通滤波
// 每 NEUQ_IMU_PERIOD_MS 毫秒调一次
void NEUQ_IMU_Update(void);

// 陀螺仪静态零偏校准，模块静止时连续采样求平均
// 500Hz下1000点就是2秒，调用期间模块必须保持静止
void NEUQ_IMU_CalibrateGyro(uint16_t samples);

// 改六轴截止频率，立即生效
void NEUQ_IMU_SetFilterHz(float acc_hz, float gyro_hz);

// 把六个滤波器的历史状态重置成当前采样值，避免出现阶跃跳变
void NEUQ_IMU_ResetFilter(void);

// 当前名义采样率 Hz
float NEUQ_IMU_GetSampleHz(void);

#endif /* _NEUQ_IMU_H_ */
