#ifndef _MOTOR_PID_H_
#define _MOTOR_PID_H_

#include "app_common.h"   /* PID_t */

void  PID_Reset(PID_t *p);                               /* 清零积分与微分历史 */
float PID_Compute(PID_t *p, float target, float actual); /* 计算输出占空比(含 bang-bang + 前馈) */

#endif /* _MOTOR_PID_H_ */
