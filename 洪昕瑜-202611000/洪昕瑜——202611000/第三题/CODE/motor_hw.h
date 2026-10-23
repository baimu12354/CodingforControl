#ifndef _MOTOR_HW_H_
#define _MOTOR_HW_H_

#include "app_common.h"

void    Motor_HW_Init(void);            /* 方向/使能/PWM/编码器/ADC 初始化 */
void    Motor_HW_SetPwm(float out);     /* 设置占空比(符号表方向), 并记录 pwm_duty */
int32_t Motor_HW_ReadEncoder(void);     /* 读编码器计数值 */
void    Motor_HW_SampleVoltage(void);   /* 供电电压采样(主循环调用, 内部 100ms 节流) */

#endif /* _MOTOR_HW_H_ */
