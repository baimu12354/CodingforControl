/* app_tick.h - 节拍接口 */

#ifndef _APP_TICK_H_
#define _APP_TICK_H_

#include "stm32f4xx_hal.h"

#define APP_FLAG_SAMPLE     (1U << 0)   /* 该采一次六轴数据 */
#define APP_FLAG_UPLOAD     (1U << 1)   /* 该往 VOFA+ 发一帧 */
#define APP_FLAG_UI         (1U << 2)   /* 该刷新一次屏幕数值 */

extern volatile uint32_t g_tick_ms;

void     AppTick_Init(void);

/* 在 PIT 中断里调用：累加毫秒、置周期标志；返回 1 表示本拍是按键扫描边界 */
uint8_t  AppTick_OnPitIsr(void);

/* 主循环取走某个周期标志，返回 1 表示该动作到点了 */
uint8_t  AppTick_TakeFlag(uint8_t flag);

uint32_t AppTick_GetMs(void);

#endif /* _APP_TICK_H_ */
