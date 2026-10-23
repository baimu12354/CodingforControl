#ifndef NEUQ_IMU_H_
#define NEUQ_IMU_H_
#include "stm32f4xx_hal.h"

// 六轴通道编号
typedef enum {
    IMU_AXIS_ACC_X = 0,   // X 轴加速度
    IMU_AXIS_ACC_Y,       // Y 轴加速度
    IMU_AXIS_ACC_Z,       // Z 轴加速度
    IMU_AXIS_GYRO_X,      // X 轴角速度
    IMU_AXIS_GYRO_Y,      // Y 轴角速度
    IMU_AXIS_GYRO_Z,      // Z 轴角速度
    IMU_AXIS_COUNT
} IMU_Axis_ENUM;

// 运行配置：滤波参数与发送轴可写入Flash，发送开关仅运行时有效
typedef struct {
    int32_t filter_k;     // 低通滤波系数 *1000，范围 0~900
    int32_t send_axis;    // 当前发送轴，见 IMU_Axis_ENUM
    int32_t send_enable;  // 波形发送开关 0/1（不写入Flash）
} IMU_Config;

extern IMU_Config imu_cfg;   // 菜单直接读写，保证页面与后台状态一致

void NEUQ_Imu_Init(void);        // MPU6050 + Flash 初始化、零偏校准、加载配置
void NEUQ_Imu_Tick(void);        // 周期中断调用：仅置采样标志
void NEUQ_Imu_Process(void);     // 主循环调用：采样/滤波/姿态解算/波形发送

float NEUQ_Imu_Raw(int32_t axis);        // 原始物理量（加速度 g，角速度 °/s）
float NEUQ_Imu_Filtered(int32_t axis);   // 低通滤波后
float NEUQ_Imu_Roll(void);               // 横滚角 °
float NEUQ_Imu_Pitch(void);              // 俯仰角 °
float NEUQ_Imu_Yaw(void);                // 航向角 °（相对初始方向）
const char *NEUQ_Imu_AxisName(int32_t axis);

void NEUQ_Imu_SaveConfig(void);          // 滤波参数/发送轴写入Flash

#endif
