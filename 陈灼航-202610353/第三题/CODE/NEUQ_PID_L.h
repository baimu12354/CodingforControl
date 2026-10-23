#ifndef _NEUQ_PID_L_H_      
#define _NEUQ_PID_L_H_

#include "NEUQ_lib_include.h"  
typedef struct {
    float Kp;            // 比例系数
    float Ki;            // 积分系数
    float Kd;            // 微分系数
    float Target;        // 目标值（这里指目标转速）
    float Actual;        // 实际值（这里指实际转速）
    float Error1;        // 本次误差 = 目标 - 实际
    float Last_Error;    // 上一次的误差（算微分项用）
    float Integral;      // 积分累加项
    float Output;        // 最终算出的输出（PWM 值）
    float Max_Output;    // 输出上限
    float Max_Integral;  // 积分限幅（防止积分饱和）
} PID_TYPEDEF;

void NEUQ_PID_INIT(PID_TYPEDEF *pid);                                  // 初始化 PID
float NEUQ_PID_CALC(PID_TYPEDEF *pid, float actual);                   // 执行一次计算
void NEUQ_PID_SETPARA(PID_TYPEDEF *pid, float kp, float ki, float kd); // 设置系数

#endif  
