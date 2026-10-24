/* led_pattern.h - LED 时序接口 */

#ifndef _LED_PATTERN_H_
#define _LED_PATTERN_H_

#include "stm32f4xx_hal.h"

void     LedPattern_Init(void);
void     LedPattern_TickIsr(void);          /* 由 PIT 中断每 10 ms 调用一次 */

void     LedPattern_SelectNext(void);       /* 待执行方案 +1 */
void     LedPattern_SelectPrev(void);       /* 待执行方案 -1 */

uint8_t  LedPattern_GetCurrent(void);       /* 当前正在执行的方案号，0 起 */
uint8_t  LedPattern_GetPending(void);       /* 待执行方案号，0 起 */

/* 界面需要重画时返回 1，取走后清 0。
 * 两种情况会置位：待执行方案被改动、周期边界上真的切换了方案。 */
uint8_t  LedPattern_TakeUiDirty(void);

/* 返回第 idx 套方案的亮/灭时长序列（毫秒），例如 "1000/1000" */
const char *LedPattern_GetSummary(uint8_t idx);

#endif /* _LED_PATTERN_H_ */
