#ifndef _NEUQ_MOTOR_H_
#define _NEUQ_MOTOR_H_

#include "NEUQ_lib_include.h"

#define MOTOR_PWM_PIN (TIM8_CH1_PC6)  //pwm引脚
#define MOTOR_DIR_PIN (PA5)  //方向引脚
#define MOTOR_PWM_FREQ (1000)  //pwm频率

void NEUQ_Motor_Init(void);  //电机初始化
void NEUQ_Motor_SetOutPut(int32_t duty);  //电机输出设置
void NEUQ_Motor_Stop(void);  //电机停止

#endif