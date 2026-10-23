#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "NEUQ_lib_include.h"

/* 接线：
   PWM_2_IN  → PA2 (TIM2_CH3)
   DIR_2_IN  → PA5
   ENCODEBL  → PB6 (TIM4 A相)
   ENCODEAL  → PB7 (TIM4 B相)
   不同板子接法存在差异，转速符号和转向相反时，把 ENC_A / ENC_B 对调 */
#define PWM_PIN   TIM2_CH3_PA2
#define DIR_PIN   PA5
#define ENC_DEV   Encoder_TIM4
#define ENC_A     Encoder_TIM4_PhaseA_PB6
#define ENC_B     Encoder_TIM4_PhaseB_PB7

void    Motor_Init(void);       // 引脚 + PWM + 编码器
void    Motor_Set(int32_t pwm); // 正=正转，负=反转，0=停
int16_t Motor_Enc(void);        // 距上次调用走过多少个计数

#endif