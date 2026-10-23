#ifndef _LCD_H
#define _LCD_H

#include "stm32f4xx_hal.h"

// 电压无效时（ADC 没接或坏了）电压行显示 N/A
#define LCD_VOLTAGE_INVALID     0xFFFFFFFF

void LCD_Menu_Init(void);

// cur/pend/on 是当前方案、待执行方案、LED 状态
// bat_mv 单位 mV，传 LCD_VOLTAGE_INVALID 就显示 N/A
// 里面自己限速了，主循环里直接反复调就行
void LCD_Menu_Update(uint8_t cur, uint8_t pend, uint8_t on, uint32_t bat_mv);

#endif /* _LCD_H */
