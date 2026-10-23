#ifndef _NEUQ_ATTITUDE_H_
#define _NEUQ_ATTITUDE_H_

#include "NEUQ_lib_include.h"

// 零偏校准默认采样次数
#define GYRO_CALIB_NUM          (500)

// 陀螺仪零偏，单位 °/s
extern float gyro_bias_x;
extern float gyro_bias_y;
extern float gyro_bias_z;

// 四元数
extern volatile float q0, q1, q2, q3;

// 欧拉角（注意：这两个是弧度，Get 系列返回度）
extern volatile float roll, pitch, yaw;

// 初始化：sampleFrequency = 调用 NEUQ_Attitude_Update 的频率(Hz)
void NEUQ_Attitude_Init(float sampleFrequency);

// 设置算法增益
void NEUQ_Attitude_SetBeta(float beta);

// 复位姿态
void NEUQ_Attitude_Reset(void);

// 固定周期调用，更新姿态
void NEUQ_Attitude_Update(void);

// 零偏校准（上电静止调一次）
void NEUQ_Gyro_Calibrate(uint16_t N);

// ★ 姿态角（度）——菜单里用这三个
float NEUQ_Attitude_GetRoll(void);
float NEUQ_Attitude_GetPitch(void);
float NEUQ_Attitude_GetYaw(void);

// 姿态角（弧度）
float NEUQ_Attitude_GetRollRadians(void);
float NEUQ_Attitude_GetPitchRadians(void);
float NEUQ_Attitude_GetYawRadians(void);

#endif