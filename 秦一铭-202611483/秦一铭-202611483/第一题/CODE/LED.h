#ifndef _LED_H_
#define _LED_H_

#include "NEUQ_lib_include.h"
#define LED_CYCLE 2000

extern volatile uint32_t cycle_timer;
extern volatile KEY_EVENT_ENUM current_mode;    //当前方案
extern volatile KEY_EVENT_ENUM pending_mode;    //待执行方案

void LED_TASK(void);       

#define LED_PIN    PA8    



#endif