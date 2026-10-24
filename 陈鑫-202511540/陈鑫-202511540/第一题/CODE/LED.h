#ifndef LED_H_
#define LED_H_

#include "NEUQ_lib_include.h"

#define LED_Pin  PA8

void LED_init(void);
void LED_Tick(void);
void scheme_update(void);

extern volatile uint8_t g_cur_scheme;
extern volatile uint8_t g_pending_scheme;
extern volatile uint8_t g_step;
extern volatile uint8_t  g_key_pressed;

#endif