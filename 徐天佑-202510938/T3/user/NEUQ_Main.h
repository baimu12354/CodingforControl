#ifndef __NEUQ_MAIN_H__
#define __NEUQ_MAIN_H__

#include <stdint.h>
#include "pid.h"

#define MOTOR_PWM  TIM8_CH1_PC6
#define MOTOR_Dir  PC7
// #define MOTOR_PWM  TIM2_CH1_PA5

// #define MOTOR_Dir  PA2
#define Circle_Count 1
#define MOTOR_ENCODER_A    Encoder_TIM3_PhaseA_PA6
#define MOTOR_ENCODER_B    Encoder_TIM3_PhaseB_PA7
// #define MOTOR_ENCODER_A    Encoder_TIM4_PhaseA_PB6
// #define MOTOR_ENCODER_B    Encoder_TIM4_PhaseB_PB7

void Motor_Control(void);
extern PID Speed;
extern volatile uint8_t Serial_RxFlag ;
extern volatile uint8_t Serial_RxPacket[16];
extern volatile uint8_t Serial_RxFlag ;
extern volatile uint8_t Serial_RxPacket[16];
extern volatile char serial_type[1];
extern volatile uint8_t motor_control_flag;
extern volatile float speed;
extern void motor_get_speed(void);
#endif
