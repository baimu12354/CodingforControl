#ifndef _NEUQ_MOTOR_H_
#define _NEUQ_MOTOR_H_

#include "NEUQ_lib_include.h"

//电机 1 
// #define MOTOR1_PWM_PIN      TIM2_CH1_PA5z

// #define MOTOR1_DIR_PIN      PA2

#define MOTOR1_PWM_PIN      TIM2_CH3_PA2     /* 调速：PA2（TIM2_CH3 = 14 ✓库支持）*/
#define MOTOR1_DIR_PIN      PA5              /* 方向：PA5（普通 GPIO ✓库支持）  */

#define MOTOR1_ENC_TIM      Encoder_TIM4

#define MOTOR1_ENC_A_PIN    Encoder_TIM4_PhaseA_PB6

#define MOTOR1_ENC_B_PIN    Encoder_TIM4_PhaseB_PB7




// //#define MOTOR1_PWM_PIN      TIM2_CH1_PA5
// #define MOTOR1_PWM_PIN      TIM3_CH2_PC7
// // #define MOTOR1_DIR_PIN      PA2
// #define MOTOR1_DIR_PIN      PC6
// // #define MOTOR1_ENC_TIM      Encoder_TIM4
// #define MOTOR1_ENC_TIM      Encoder_TIM3
// // #define MOTOR1_ENC_A_PIN    Encoder_TIM4_PhaseA_PB6
// #define MOTOR1_ENC_A_PIN    Encoder_TIM3_PhaseA_PA6
// // #define MOTOR1_ENC_B_PIN    Encoder_TIM4_PhaseB_PB7
// #define MOTOR1_ENC_B_PIN    Encoder_TIM3_PhaseB_PA7

//极性约定
#define MOTOR1_DIR_FORWARD   GPIO_High
#define MOTOR1_DIR_BACKWARD  GPIO_Low

//PID参数
// #define MOTOR_KP            0.0f
// #define MOTOR_KI            10.0f
// #define MOTOR_KD            0.0f
#define MOTOR_DUTY_STEP     500.0f

#define CTRL_PERIOD_S       0.01f       //控制周期 10ms
#define ENC_PPR             11          //编码器线数
#define ENC_MULT            4           //倍频数
#define GEAR_RATIO          10          //减速比

//轴转一圈的脉冲个数
#define ENC_CNT_PER_REV     (ENC_PPR * ENC_MULT * GEAR_RATIO)

//每个脉冲代表多少 r/s
#define SPEED_SCALE         (1.0f / ENC_CNT_PER_REV / CTRL_PERIOD_S)

//编码器方向
#define MOTOR1_ENC_SIGN     (-1.0f)


extern float g_kp;
extern float g_ki;
extern float g_kd;
extern float speed_aim;
extern float speed_now;
extern int   g_run;
extern int duty_now;

void NEUQ_Motor_Init(void);
void NEUQ_Motor_Control(void);

#endif