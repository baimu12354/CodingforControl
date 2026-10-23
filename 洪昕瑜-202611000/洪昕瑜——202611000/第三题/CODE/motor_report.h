#ifndef _MOTOR_REPORT_H_
#define _MOTOR_REPORT_H_

#include "app_common.h"

void Report_Tick(void);   /* 周期上报(内部按 REPORT_PERIOD_MS 节流) */

#endif /* _MOTOR_REPORT_H_ */
