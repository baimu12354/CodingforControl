#ifndef _NEUQ_MOTOR_H_
#define _NEUQ_MOTOR_H_

#define MOTOR_PWM_PIN  (TIM8_CH1_PC6)
#define MOTOR_DIR_PIN  (PC7)

void NEUQ_Motor_Init(void);
void NEUQ_Motor_SetDuty(int duty);



#endif