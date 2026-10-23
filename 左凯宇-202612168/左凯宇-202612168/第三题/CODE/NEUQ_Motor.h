#ifndef _NEUQ_MOTOR_H_
#define _NEUQ_MOTOR_H_
#include "NEUQ_lib_include.h"

typedef struct {
    float speed;//实际
    float motor_duty;//占空比
    float set_spd;//设定
    float les_spd;//差值
    float les_spd_last;//上次差值
    float speed_last;//上次速度
    float les_spd_total;//差值总和
    float kp;
    float ki;
    float kd;
    int32_t Max_I;
    uint8_t motor_run;
}NEUQ_Motor_params;

extern NEUQ_Motor_params params;

#define  Motor_PWM_R  TIM2_CH3_PA2
#define  Motor_Dir    PA5

void NEUQ_Motor_con_init(void);
void NEUQ_Motor_con(void);

#define ENCODER_LINES   11
#define ENCODER_MULT    4
#define GEAR_RATIO      10
#define CPR_OUTPUT      (ENCODER_LINES * ENCODER_MULT * GEAR_RATIO)   // 440
#define RPM_TO_CNT10MS(r)   ((float)(r) * (float)CPR_OUTPUT / 6000.0f)
#define CNT10MS_TO_RPM(c)   ((float)(c) * 6000.0f / (float)CPR_OUTPUT)


#endif