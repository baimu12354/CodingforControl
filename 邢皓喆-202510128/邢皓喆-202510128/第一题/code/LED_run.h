#ifndef _LED_run_H
#define _LED_run_H

#include "NEUQ_lib_include.h"
extern volatile uint8_t TASK_CMD;
typedef enum{
    LED_MODE_SLOW_BLINK = 1 ,// 慢闪：亮 1000ms，灭 1000ms
    LED_MODE_SINGLE_FLASH = 2 ,// 单闪：亮 200ms，灭 1800ms
    LED_MODE_DOUBLE_FLASH = 3,  // 双闪：亮 200ms，灭 200ms，亮 200ms，灭 1400ms
    LED_MODE_SHORT_LONG_FLASH = 4  // 短长闪：亮 200ms，灭 200ms，亮 600ms，灭 1000ms
}LED_MODE_ID;

void LED_RUN_init(void);
void LED_TASK(LED_MODE_ID id);

#endif
