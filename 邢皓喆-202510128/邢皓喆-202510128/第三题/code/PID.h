#ifndef _PID_H_
#define _PID_H_


#define PWM_MAXPWM_MAX 1000;
#include "NEUQ_lib_include.h"
extern float pwm_value;
extern volatile float target;
extern volatile float actural;
extern volatile uint8_t motor_run;
extern IncrementalPID speed_pid;
extern volatile uint8_t motor_stop_pending;
float PWM_value(IncrementalPID *pid, float tar_speed, float cur_speed);
void PID_UPDATE(void);
#endif
