#ifndef _NEUQ_Motor_H_
#define _NEUQ_Motor_H_

#include "NEUQ_lib_include.h"

#define MOTOR_PWM_PIN     TIM8_CH1_PC6      // PWM 输出引脚
#define MOTOR_DIR1_PIN    PE10              // 方向1
#define MOTOR_DIR2_PIN    PE11              // 方向2     

void Motor_Init(void);
void Motor_SetOutput(int32_t out);
void NEUQ_Motor_Stop(void);

#endif



