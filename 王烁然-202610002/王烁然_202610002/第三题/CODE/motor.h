#ifndef _MOTOR_H_
#define _MOTOR_H_

//有的板C7C6是反的，测试的时候forward写1，如果电机玩命转试着调换。大部分c和a在同一边
//实测forward是0，测试时先写1看看有没有接反
#define MOTOR_PWM_PIN TIM8_CH1_PC6
#define MOTOR_DIR_PIN PC7
#define MOTOR_DIR_FORWAR 0
#define MOTOR_DIR_BACK 1
#define ENCODER_TIMER            Encoder_TIM4
#define ENCODER_PHRASE_A_PIN     Encoder_TIM4_PhaseA_PB6
#define ENCODER_PHRASE_B_PIN     Encoder_TIM4_PhaseB_PB7

// #define ENCODER_TIMER Encoder_TIM4
// #define ENCODER_PHRASE_A_PIN Encoder_TIM4_PhaseA_PB6
// #define ENCODER_PHRASE_B_PIN Encoder_TIM4_PhaseB_PB7

#define RATE 142   //编码器*RATE=PWM

#include "NEUQ_lib_include.h"

typedef struct 
{
    // 通信/控制目标
    int16_t target;      // 目标转速（上位机下发）
    int16_t actual;      // 实际转速（编码器算出来）
    uint8_t  run;            // 启停标志：0停止 1运行
    int16_t PWM_Record;//pid算出来要应用的pwm，有正负，喂给setduty函数
    // PID 参数（上位机可改）
    float Kp;
    float Ki;
    float Kd;

    // PID 运算用到的“历史状态”（算PID必须存）
    float error;
    float last_error;
    float last_last_error;
    float integral;
    float output;
    /* data */
}Motor_PID_Data;

extern volatile Motor_PID_Data Motor_Data;

void Motor_Inite(void);

void Motor_SetDuty(int16_t duty);

void Motor_PIDoutput(void);

#endif // !_MOTOR_H_




