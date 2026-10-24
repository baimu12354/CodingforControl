/* app_tick.h - 节拍接口 */

#ifndef _APP_TICK_H_
#define _APP_TICK_H_

#include "stm32f4xx_hal.h"

extern volatile uint32_t g_tick_10ms;       /* 10 ms 节拍计数，仅中断写入 */

void     AppTick_Init(void);
void     AppTick_OnPitInterrupt(void);      /* 在 PIT 回调里调用 */
uint32_t AppTick_Get(void);                 /* 读取当前节拍 */

#endif /* _APP_TICK_H_ */
