#ifndef _NEUQ_TASK3_H_
#define _NEUQ_TASK3_H_

#include "NEUQ_lib_include.h"

// ========== 位置式 PID 结构体 ==========
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float integ;        // 积分累加项（已经乘过 Ki）
    float err_prev;     // 上一次误差
    float out_max;      // 输出上限
    float out_min;      // 输出下限
} PosPID_t;

// ========== 全局变量声明 ==========
extern PosPID_t g_pos_pid;                 // 位置式 PID 实例
extern volatile float g_target_rpm;        // 目标转速
extern volatile uint8_t g_motor_running;   // 电机运行标志
extern int32_t g_pwm_output;               // 当前 PWM 输出

// ========== 函数声明 ==========
void NEUQ_Task3_Init(void);
void NEUQ_Task3_Task(void);
void NEUQ_Task3_Tick_10ms(void);

// 位置式 PID 复位（清积分和历史误差）
void PosPID_Reset(PosPID_t *pid);

// 位置式 PID 计算，返回 PWM 值
float PosPID_Update(PosPID_t *pid, float target, float actual);

#endif