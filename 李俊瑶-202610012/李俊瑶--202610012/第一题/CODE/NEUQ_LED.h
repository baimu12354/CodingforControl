#ifndef _NEUQ_LED_H_
#define _NEUQ_LED_H_

#include "NEUQ_lib_include.h"

#define NEUQ_LED_PLAN_COUNT 4  

void NEUQ_Led_Init(void);
void NEUQ_Led_Task(void);
void NEUQ_Led_Tick_Handler(void);

void NEUQ_Led_SetPendingPlan(uint8_t plan);
extern volatile uint32_t led_tick ;

uint8_t NEUQ_Led_GetCurrentPlan(void);
uint8_t NEUQ_Led_GetPendingPlan(void);


#endif
