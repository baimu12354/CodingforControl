#ifndef LED_DISPLAY_H
#define LED_DISPLAY_H

/* LED 菜单静态绘制与动态数据显示。 */
#include <stdint.h>
void Led_Display_Init(void);
void Led_Display_Refresh(uint8_t current, uint8_t pending, uint32_t battery_mv);

#endif /* LED_DISPLAY_H */
