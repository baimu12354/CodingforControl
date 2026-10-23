#ifndef _NEUQ_IMU_H_
#define _NEUQ_IMU_H_

#include "NEUQ_lib_include.h"
extern float filtered_acc_x;  // 滤波后加速度 X (g)
extern float filtered_acc_y;  // 滤波后加速度 Y (g)
extern float filtered_acc_z;  // 滤波后加速度 Z (g)
extern float filtered_gyro_x; // 滤波后角速度 X (dps)
extern float filtered_gyro_y; // 滤波后角速度 Y (dps)
extern float filtered_gyro_z; // 滤波后角速度 Z (dps)
extern float attitude_pitch;
extern float attitude_roll;
extern float attitude_yaw;

/* 姿态四元数（机体坐标系 -> 世界坐标系，单位四元数，每次更新后归一化） */
extern float imu_q0; // w
extern float imu_q1; // x
extern float imu_q2; // y
extern float imu_q3; // z

/* 陀螺仪零偏 */
extern float gyro_bias_x;
extern float gyro_bias_y;
extern float gyro_bias_z;

/* 最近一次姿态更新的积分步长（ms），正常为 10ms（100Hz） */
extern float imu_dt_ms;

typedef enum {
    IMU_CALIB_IDLE = 0,// 未校准
    IMU_CALIB_RUNNING,// 正在校准（此时姿态解算暂停）
    IMU_CALIB_OK,// 校准成功
    IMU_CALIB_FAIL,// 校准失败（模块未静止），可重新校准
} IMU_CALIB_STATE;

extern volatile uint8_t imu_calib_state;
extern volatile uint8_t imu_calib_percent; // 校准进度 0~100

void NEUQ_IMU_Init(void);// 上电初始化 + 静态零偏校准
void MPU6050_Filter_Task(void);// 六轴采集与一阶低通滤波（10ms）
void Attitude_Solve_Task(void);// 四元数姿态解算（10ms）

void NEUQ_IMU_StartCalib(void);// 请求重新静态校准（非阻塞）
uint8_t NEUQ_IMU_CalibTask(void);// 校准状态机，运行中返回 1
uint8_t NEUQ_IMU_IsCalibRunning(void);

#endif
