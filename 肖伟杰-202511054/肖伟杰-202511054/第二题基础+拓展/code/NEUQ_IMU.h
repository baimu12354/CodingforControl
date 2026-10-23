/*********************************************************************************************************************
* 文件名称          NEUQ_IMU
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第二题：MPU6050 六轴采集 + 六轴独立滤波 + 静态零偏校准
*
*   六个轴每 5 ms 采一次，一个轴一个一阶 RC 低通（状态互不影响）：
*       y += alpha * (x - y)        alpha = 1 - exp(-2*pi*fc*dt)   （见 NEUQ_FilterCoef.h）
*   滤波在原始计数上做，所以 VOFA+ 的"原始"和"滤波"两个通道量纲完全一样。
*
*   采集和滤波都在 TIM7 的 5 ms 中断里（见 User/NEUQ_Interrupts.c），
*   刷屏和串口在主循环 —— 所以切页面不会停止采集与滤波。
********************************************************************************************************************/
#ifndef NEUQ_IMU_H_
#define NEUQ_IMU_H_

#include "stm32f4xx_hal.h"
#include "NEUQ_MPU6050.h"       /* 提供 mpu6050_acc_x 等六轴原始量与换算函数 */

#define IMU_PERIOD_MS       5U      /* 采集/姿态节拍，200 Hz */
#define IMU_CALIB_CNT       300U    /* 零偏校准样本数：300 × 5 ms = 1.5 s */

/* ------------------------- 六个轴 ------------------------- */
typedef enum
{
    AXIS_ACC_X = 0, AXIS_ACC_Y, AXIS_ACC_Z,
    AXIS_GYR_X,     AXIS_GYR_Y, AXIS_GYR_Z,
    AXIS_NUM
} NEUQ_AxisEnum;

extern volatile float imu_raw[AXIS_NUM];      /* 原始计数 */
extern volatile float imu_filt[AXIS_NUM];     /* 滤波值（同为原始计数） */
extern volatile float imu_mg[3];              /* 加速度，mg（静止 Z ≈ +1000） */
extern volatile float imu_cdps[3];            /* 角速度，0.01 dps */
extern volatile float imu_bias[3];            /* 陀螺零偏，0.01 dps */

uint8_t NEUQ_IMU_Init(void);                  /* 1 = 找到 MPU6050 */
void    NEUQ_IMU_SetFc(uint16_t fc_x10);      /* 设截止频率，传 0.1Hz（如 100 = 10.0Hz） */
void    NEUQ_IMU_Tick(void);                  /* 5 ms 中断调用：采集 + 滤波 + 校准累计 + 姿态 */
void    NEUQ_IMU_Calib(void);                 /* 启动零偏校准 */
uint8_t NEUQ_IMU_CalibBusy(void);             /* 1 = 校准还没做完 */

/* 1 = MPU6050 在线（初始化成功） */
uint8_t NEUQ_IMU_Online(void);

#endif /* NEUQ_IMU_H_ */
