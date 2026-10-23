#ifndef NEUQ_LED_H_
#define NEUQ_LED_H_

#include "NEUQ_lib_include.h"

//板上 LED 实际连接的引脚
#define LED_PIN       PC0

#define LED_ON_LEVEL  GPIO_High
#define LED_OFF_LEVEL GPIO_Low

void LED_Init(void);     // 初始化 LED 引脚（初始为熄灭状态）
void LED_On(void);       // 点亮小灯
void LED_Off(void);      // 熄灭小灯
void LED_Toggle(void);   // 翻转小灯（亮变灭、灭变亮）

// ==================== LED 闪烁方案（每套完整周期 2s） ====================
void     LED_Scheme_Init(void);            // 上电初始化：当前/待执行方案都置为 0
void     LED_Scheme_Tick10ms(void);        // 每 10ms 调用一次（PIT 中断）：推进周期并刷新 LED
void     LED_Scheme_SetPending(uint8_t s); // 修改待执行方案（0~3），下一周期起点生效
uint8_t  LED_Scheme_GetCurrent(void);      // 读当前方案（0~3，显示用）
uint8_t  LED_Scheme_GetPending(void);      // 读待执行方案（0~3，显示用）
uint32_t LED_GetTick10ms(void);            // 读 10ms 系统计数（菜单刷新节流用）

#endif
