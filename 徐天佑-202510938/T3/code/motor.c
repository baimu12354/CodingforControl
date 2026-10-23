#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "pid.h"
#include "NEUQ_Main.h"
#include <stdlib.h>
volatile uint8_t motor_control_flag = 0;
volatile float speed=0;
uint16_t count=0;

void Motor_Control(void)
{
 
    float duty;


    pid_update(&Speed);
    duty = Speed.output;
    if (duty >= 0)
    {
        NEUQ_GPIO_write(MOTOR_Dir, GPIO_High);
    }
    else
    {
        NEUQ_GPIO_write(MOTOR_Dir, GPIO_Low);

    }

    if (duty > PWM_MAX){
        duty = PWM_MAX;}
    if(duty< -PWM_MAX){
        duty = -PWM_MAX;}

    NEUQ_PWM_set_duty( MOTOR_PWM, fabsf(duty));
}


void motor_get_speed(void) {

    int16_t raw_speed = NEUQ_Encoder_Read(Encoder_TIM3);


    NEUQ_Encoder_Clear(Encoder_TIM3);

    speed =- (float)raw_speed;


    Speed.actual = speed; 
}
