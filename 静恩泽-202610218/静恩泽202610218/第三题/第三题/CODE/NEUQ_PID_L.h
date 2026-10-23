#ifndef _NEUQ_PID_L_H_
#define _NEUQ_PID_L_H_

#include "NEUQ_lib_include.h"

/* ==========================================================================
 * PID 控制器结构体：保存三个系数 + 计算过程中需要的中间量
 * ========================================================================== */
typedef struct {
    float Kp;            // 比例系数
    float Ki;            // 积分系数
    float Kd;            // 微分系数
    float Target;        // 目标值（这里指目标转速）
    float Actual;        // 实际值（这里指实际转速）
    float Error1;        // 本次误差 = 目标 - 实际
    float Last_Error;    // 上一次误差 e(k-1)
    float Prev_Error;    // 上上次误差 e(k-2)
    float Output;        // 累加后的最终输出（PWM 值）
    float Max_Output;    // 输出上限
} PID_TYPEDEF;

/* ==========================================================================
 * 三个函数声明，实现见 NEUQ_PID_L.c
 * ========================================================================== */
void NEUQ_PID_INIT(PID_TYPEDEF *pid);                                  // 初始化 PID
float NEUQ_PID_CALC(PID_TYPEDEF *pid, float actual);                   // 执行一次计算
void NEUQ_PID_SETPARA(PID_TYPEDEF *pid, float kp, float ki, float kd); // 设置系数

#endif
