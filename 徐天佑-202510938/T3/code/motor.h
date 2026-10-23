#ifndef __MOTOR_H__
#define __MOTOR_H__
extern volatile uint8_t motor_control_flag ;
extern volatile float speed;
extern int32_t count;


void Motor_Control(void);


void motor_get_speed(void);
#endif