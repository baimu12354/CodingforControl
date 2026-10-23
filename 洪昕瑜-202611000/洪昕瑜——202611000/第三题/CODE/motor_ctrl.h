#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include "app_common.h"

/* 对外接口 (User_Main 调用) */
void MotorCtrl_Init(void);            /* 初始化硬件与控制参数(在 User_Main 中调用一次) */
void MotorCtrl_PeriodCallback(void);  /* 1ms 控制周期回调(挂到 NEUQ_PIT_1_CALLBACK) */
void MotorCtrl_RxByte(uint8_t dat);   /* 串口接收字节入口(挂到 TS_UART_0_RX_CALLBACK) */
void MotorCtrl_Loop(void);            /* 主循环调用(帧解析 + 周期上报 + 电压采样) */

#endif /* _MOTOR_CTRL_H_ */
