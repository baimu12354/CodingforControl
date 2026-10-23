#ifndef _LED_H_
#define _LED_H_
#include"NEUQ_lib_include.h"

void led_init(void);
void led_task(KEY_EVENT_ENUM key2);
void led_pit_callback(void);
void led_set_mode(uint16_t mode);


#include"NEUQ_lib_include.h"

void led_init(void);
void led_task(KEY_EVENT_ENUM key2);
void led_pit_callback(void);

#endif