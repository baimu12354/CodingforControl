#ifndef _MOTOR_SPEED_H_
#define _MOTOR_SPEED_H_

#include"NEUQ_lib_include.h"
#include"NEUQ_PID.h"
typedef enum{
motor_stop=0,
motor_run=1,


}MotorState_enum;
extern pid_value speed_pid;
extern float target_speed;
extern float real_speed;
extern MotorState_enum motor_state;

void motor_speed_init(void);
float encoder_getspeed(void);
void motor_setpwm(float duty);

#endif