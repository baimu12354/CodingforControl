#ifndef _mpu6050_H
#define _mpu6050_H

#include "NEUQ_lib_include.h"
extern float mpu_filter_alpha;
extern uint8_t filter_select;
extern float raw[6];
extern float filterd[6];
extern volatile uint32_t timer_ms;
typedef enum
{
 ON = 1,
 OFF = 0
}Upload_State;
extern uint8_t upload_enable;
void MPU6050_Task(void);
uint32_t TIMER_GET_MS(void);
void MPU6050_Service(void);
#endif
