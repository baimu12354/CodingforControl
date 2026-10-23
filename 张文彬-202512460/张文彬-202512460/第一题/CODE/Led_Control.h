#ifndef LED_CONTROL_H
#define LED_CONTROL_H

/* 四套 LED 时序表及电平输出，周期切换由主循环负责。 */
#include <stdint.h>
void Led_Control_Init(void);
void Led_Control_Update(uint8_t scheme, uint32_t elapsed_ms);

#endif /* LED_CONTROL_H */
