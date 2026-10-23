#ifndef _NEUQ_IMU_H_
#define _NEUQ_IMU_H_

#include "NEUQ_lib_include.h"

extern float filtered_acc_x;  // 滤波后加速度 X
extern float filtered_acc_y;  // 滤波后加速度 Y
extern float filtered_acc_z;  // 滤波后加速度 Z
extern float filtered_gyro_x; // 滤波后角速度 X
extern float filtered_gyro_y; // 滤波后角速度 Y
extern float filtered_gyro_z; // 滤波后角速度 Z
extern float attitude_pitch;  // 姿态角：俯仰角
extern float attitude_roll;   // 姿态角：横滚角
extern float attitude_yaw;    // 姿态角：偏航角

void Attitude_Solve_Task(void);       // 姿态解算任务
void MPU6050_Filter_Task(void);       // 采集与滤波任务
void NEUQ_IMU_Init(void);
#endif