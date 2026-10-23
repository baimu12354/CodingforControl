#ifndef __NEUQ_MOTOR_H_
#define __NEUQ_MOTOR_H_

#include "NEUQ_lib_include.h"

typedef struct
{
    IncrementalPID pid;
    float target_rpm;
    float current_rpm;
    int8_t running;

} MotorData;

#define Alpha 0.10    //滤波参数
extern volatile MotorData motor ;
extern volatile int32_t motor_ms;

static int32_t last_pick;
static int32_t now;


void MOTOR_Init(void);
void MOTOR_Start(void);
void MOTOR_Stop(void);  
void MOTOR_Set_PID(float PID_P,float PID_I,float PID_D);    
void MOTOR_Set_RPM(float RPM);        
void MOTOR_Task(void);                                                   
#endif