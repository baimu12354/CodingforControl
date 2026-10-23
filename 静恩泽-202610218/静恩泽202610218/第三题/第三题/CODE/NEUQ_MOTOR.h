#ifndef _NEUQ_MOTOR_H_
#define _NEUQ_MOTOR_H_

#include "NEUQ_lib_include.h"

void NEUQ_MOTOR_INIT(void);                              // 初始化：PWM、方向脚、PID
void NEUQ_MOTOR_Start(void);                             // 启动电机
void NEUQ_MOTOR_Stop(void);                              // 停止电机
void NEUQ_MOTOR_SetTarget(float speed);                  // 设置目标转速（圈/秒，正负=正反转）
void NEUQ_MOTOR_SetPID(float Kp, float Ki, float Kd);    // 一次设置 PID 三个参数
void NEUQ_MOTOR_SetKp(float Kp);                         // 单独设置 Kp
void NEUQ_MOTOR_SetKi(float Ki);                         // 单独设置 Ki
void NEUQ_MOTOR_SetKd(float Kd);                         // 单独设置 Kd
void NEUQ_MOTOR_Task(void);                              // 控制主任务（10ms 调用一次）

float   NEUQ_MOTOR_GetTarget(void);   // 读取当前目标转速
float   NEUQ_MOTOR_GetActual(void);   // 读取当前实际转速
float   NEUQ_MOTOR_GetDuty(void);     // 读取当前实际输出的 PWM 量（0~10000，调试用）
float   NEUQ_MOTOR_GetKp(void);       // 读取当前 Kp
float   NEUQ_MOTOR_GetKi(void);       // 读取当前 Ki
float   NEUQ_MOTOR_GetKd(void);       // 读取当前 Kd
uint8_t NEUQ_MOTOR_IsRunning(void);   // 查询是否在运行

#endif
