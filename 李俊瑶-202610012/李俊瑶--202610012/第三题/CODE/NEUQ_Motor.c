#include "NEUQ_lib_include.h"

void NEUQ_Motor_Init(void)
{
    NEUQ_GPIO_init(MOTOR_DIR_PIN1, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_PWM_init(MOTOR_PWM_PIN1, 1000, 0);
    NEUQ_Encoder_init(Encoder_TIM4, Encoder_TIM4_PhaseA_PB6,
                      Encoder_TIM4_PhaseB_PB7);
}

void NEUQ_Motor_SetDuty(int duty)
{
    if (duty > 10000) duty = 10000;
    if (duty < -10000) duty = -10000;

    if (duty >= 0) {
        NEUQ_GPIO_write(MOTOR_DIR_PIN1, GPIO_Low);
        NEUQ_PWM_set_duty(MOTOR_PWM_PIN1, duty);
    } else {
        NEUQ_GPIO_write(MOTOR_DIR_PIN1, GPIO_High);
        NEUQ_PWM_set_duty(MOTOR_PWM_PIN1, -duty);
    }
}
