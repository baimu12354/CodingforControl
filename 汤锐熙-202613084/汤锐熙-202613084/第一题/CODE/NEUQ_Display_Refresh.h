#ifndef _NEUQ_Display_Refresh_
#define _NEUQ_Display_Refresh_

#include "NEUQ_lib_include.h"

/* ================= 任务参数（按实际硬件修改） ================= */
#define LED_PIN      PA8      // LED 引脚，高电平点亮

// 四个按键引脚（占位值，定了实际接线后替换；按下 = 低电平）
#define BTN1_PIN     PE7
#define BTN2_PIN     PE8
#define BTN3_PIN     PE9
#define BTN4_PIN     PE10

#define DEBOUNCE_MS  20       // 按键消抖时间
#define CYCLE_MS     2000     // 每套动作完整周期 2s

/* ================= 共享状态 =================
 * 由 1ms 中断更新，主循环 / 屏幕只读取。
 */
extern volatile uint8_t g_led_current;   // 当前方案 1~4
extern volatile uint8_t g_led_next;      // 待执行方案 1~4
extern volatile uint8_t g_led_dirty;     // 屏幕脏标志：1 = 需要刷新

/* ================= 屏幕刷新 ================= */
void NEUQ_Display_Refresh_Init(void);    // 上电画静态文字 + 初值
void NEUQ_Display_Refresh_Update(void);  // 仅更新两个方案数字

#endif
