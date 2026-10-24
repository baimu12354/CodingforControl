#include "NEUQ_lib_include.h"


void NEUQ_Motor_Init(void){
    NEUQ_PWM_init(MOTOR_PWM_PIN, 1000, 0);
    NEUQ_GPIO_init(MOTOR_DIR_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_Encoder_init(Encoder_TIM3, Encoder_TIM3_PhaseA_PA6, Encoder_TIM3_PhaseB_PA7);

}

void NEUQ_Motor_SetDuty(int duty){
    if(duty>=10000){
        duty = 10000;
    }
    if(duty<=-10000){
        duty = -10000;
    }
    if(duty>=0){
        NEUQ_PWM_set_duty(MOTOR_PWM_PIN, duty);
        NEUQ_GPIO_write(MOTOR_DIR_PIN, GPIO_High);
    }
    if(duty<0){
        NEUQ_PWM_set_duty(MOTOR_PWM_PIN, -duty);
        NEUQ_GPIO_write(MOTOR_DIR_PIN, GPIO_Low);
    }

}