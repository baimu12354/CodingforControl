#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "NEUQ_lib_include.h"

#define MOTOR1_PWM     TIM2_CH3_PA2
#define MOTOR1_DIR     PA5
#define MOTOR1_ENC     Encoder_TIM3
#define MOTOR1_ENC_A   Encoder_TIM3_PhaseA_PA6
#define MOTOR1_ENC_B   Encoder_TIM3_PhaseB_PA7

#define MOTOR2_PWM     TIM8_CH1_PC6
#define MOTOR2_DIR     PC7
#define MOTOR2_ENC     Encoder_TIM4
#define MOTOR2_ENC_A   Encoder_TIM4_PhaseA_PB6
#define MOTOR2_ENC_B   Encoder_TIM4_PhaseB_PB7

#define MOTOR_PWM_FREQ      8000u
#define MOTOR_ENCODER_CPR   440u
#define MOTOR_CTRL_PERIOD_S 0.005f
#define MOTOR_COUNT         2

#define MOTOR_DIR_FIX       (1)

typedef struct {
    float target_rpm, actual_rpm;
    float Kp, Ki, Kd;     
    float integral;
    float last_error;
} Motor_t;

extern Motor_t motor[MOTOR_COUNT];
extern uint8_t motor_enable;

void  Motor_Init(void);
void  Motor_Control_Loop(void);
void  Motor_Task(void);
void  Motor_SetEnable(uint8_t en);
void  Motor_SetTarget(uint8_t idx, float rpm);
void  Motor_SetPID(float kp, float ki, float kd);

float Motor_GetTarget(uint8_t idx);
float Motor_GetActual(uint8_t idx);
float Motor_GetKp(void);
float Motor_GetKi(void);
float Motor_GetKd(void);

#endif