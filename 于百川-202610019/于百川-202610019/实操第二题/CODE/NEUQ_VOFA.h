#ifndef _NEUQ_SENSOR_H_
#define _NEUQ_SENSOR_H_

#include "NEUQ_lib_include.h"

/* VOFA+ 使用的串口 */
#define VOFA_UART   UART_0

/* 轴编号：0..2=加速度X/Y/Z，3..5=角速度X/Y/Z（对应 Params.send_axis） */
#define AXIS_ACC_X   0
#define AXIS_ACC_Y   1
#define AXIS_ACC_Z   2
#define AXIS_GYRO_X  3
#define AXIS_GYRO_Y  4
#define AXIS_GYRO_Z  5

/* 1ms 系统节拍（在 PIT_1 中断里累加），供动态页 100ms 节拍刷新用 */
extern volatile uint32_t NEUQ_sys_tick;

/* 滤波后的六轴数据：加速度单位 g，角速度单位 °/s（供 LCD 显示 / VOFA+ 发送） */
extern float acc_filt[3];
extern float gyro_filt[3];

void NEUQ_Sensor_Init(void);          /* MPU6050 + VOFA+ 串口初始化（须在 Menu_Init 之后调用） */
void NEUQ_Sensor_Task(void);          /* 10ms 节拍：采样→滤波→姿态→VOFA+发送 */
void NEUQ_Sensor_SetAlpha(int32_t alpha_pct);  /* 设置滤波系数（5..95，与菜单一致） */

#endif
