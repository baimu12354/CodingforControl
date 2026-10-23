#ifndef LED_CONFIG_H
#define LED_CONFIG_H

/* LED、按键、ADC 引脚及原有时间参数。 */
#include <stdint.h>

#define APP_LED_PIN                 PA8
#define APP_KEY_PIN                 PA0
#define APP_ADC_CHANNEL             ADC1_CH3_PA3

#define APP_ADC_REFERENCE_MV        3300UL
#define APP_ADC_FULL_SCALE          4095UL
/* 电池经 3:1/1:1 分压后接 PA3，换算倍数为 4。若实物阻值不同，只改这里。 */
#define APP_DIVIDER_NUMERATOR       4UL
#define APP_DIVIDER_DENOMINATOR     1UL

#define APP_PERIOD_MS               2000UL
#define APP_LCD_REFRESH_MS          100UL
#define APP_KEY_DEBOUNCE_MS         20UL
//注意此处的UL是无无符号整型号，即在宏定义里面先定义该量的符号

#endif /* LED_CONFIG_H */
