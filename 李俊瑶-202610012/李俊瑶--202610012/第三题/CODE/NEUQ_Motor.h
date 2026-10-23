#ifndef _NEUQ_MOTOR_H_
#define _NEUQ_MOTOR_H_

#include <stdint.h>

#define MOTOR_PWM_PIN1 (TIM8_CH1_PC6)
#define MOTOR_DIR_PIN1 (PC7)
#define MOTOR_CONTROL_PERIOD_MS (10U)
#define MOTOR_ENCODER_COUNTS_PER_REV (0.0f)
#define MOTOR_ENCODER_SIGN (1)

extern volatile float target_speed;
extern volatile float current_pwm;
extern volatile uint8_t motor_enable;
extern volatile float g_actual_speed;
extern volatile uint8_t telemetry_ready;

void NEUQ_Motor_Init(void);
void NEUQ_Motor_SetDuty(int duty);

#endif
