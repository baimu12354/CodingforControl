/*********************************************************************************************************************
* 文件名称          NEUQ_Attitude
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第二题（挑战 20 分）：四元数姿态解算
*
*   四元数互补滤波，每 5 ms 一次：
*     ① 用四元数估计重力方向     v = R(q)ᵀ·[0,0,1]
*     ② 实测重力方向 a（加速度计归一化），误差 e = a × v
*     ③ e 乘 Kp 去修正角速度     （陀螺动态准、加速度计长期准，这就是"互补"）
*     ④ 四元数微分               q̇ = 0.5·q ⊗ ω
*     ⑤ 归一化                   （挑战题明确要求）
*     ⑥ 四元数 → 横滚/俯仰/航向
*
*   坐标轴：MPU6050 丝印 X/Y/Z 右手系。水平放置时 az ≈ +1 g，此时三轴角都是 0°。
*   单位：deg。航向以"校准那一刻"为 0°，只反映相对转动。
********************************************************************************************************************/
#ifndef NEUQ_ATTITUDE_H_
#define NEUQ_ATTITUDE_H_

#include "stm32f4xx_hal.h"

extern volatile float att_roll;      /* 横滚 deg */
extern volatile float att_pitch;     /* 俯仰 deg */
extern volatile float att_yaw;       /* 航向 deg（相对初始方向） */

void NEUQ_Attitude_Init(void);
void NEUQ_Attitude_Reset(void);      /* 用当前重力方向把姿态角置零 */

/* 输入：角速度 dps（已扣零偏）、加速度 g。dt 固定 5 ms，写在本文件里 */
void NEUQ_Attitude_Update(float gx, float gy, float gz,
                          float ax, float ay, float az);

#endif /* NEUQ_ATTITUDE_H_ */
