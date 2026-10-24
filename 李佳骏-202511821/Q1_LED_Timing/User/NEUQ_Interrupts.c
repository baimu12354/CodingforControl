/*
 * 基于 NEUQ 开源库（GPLv3）。
 * Copyright (c) 2025 NEUQ SmartCar Lab
 * 许可证：https://www.gnu.org/licenses/gpl-3.0.html
 */

/* 第一题：LED 时序控制，独立固件，只用 LCD、按键、LED 和 PIT。 */

#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "../CODE/app_tick.h"
#include "../CODE/bsp_key.h"
#include "../CODE/led_pattern.h"

/* 10 ms PIT：节拍 + 按键扫描 + LED 时序 */
void NEUQ_PIT_1_CALLBACK(void)
{
    AppTick_OnPitInterrupt();
    Key_Scan10ms();
    LedPattern_TickIsr();
}

void NEUQ_PIT_2_CALLBACK(void)
{
    /* 第一题只用 PIT_CH1，这里保持空实现 */
}

/* 串口接收回调：本题不用 */
void TS_UART_0_RX_CALLBACK(uint8_t dat)
{
    (void)dat;
}

void TS_UART_1_RX_CALLBACK(uint8_t dat)
{
    (void)dat;
}

void TS_UART_2_RX_CALLBACK(uint8_t dat)
{
    (void)dat;
}

void TS_UART_3_RX_CALLBACK(uint8_t dat)
{
    (void)dat;
}

/* 外部中断回调：本题不用 */
void NEUQ_EXTI0_Callback(void)  { }
void NEUQ_EXTI1_Callback(void)  { }
void NEUQ_EXTI2_Callback(void)  { }
void NEUQ_EXTI3_Callback(void)  { }
void NEUQ_EXTI4_Callback(void)  { }
void NEUQ_EXTI5_Callback(void)  { }
void NEUQ_EXTI6_Callback(void)  { }
void NEUQ_EXTI7_Callback(void)  { }
void NEUQ_EXTI8_Callback(void)  { }
void NEUQ_EXTI9_Callback(void)  { }
void NEUQ_EXTI10_Callback(void) { }
void NEUQ_EXTI11_Callback(void) { }
void NEUQ_EXTI12_Callback(void) { }
void NEUQ_EXTI13_Callback(void) { }
void NEUQ_EXTI14_Callback(void) { }
void NEUQ_EXTI15_Callback(void) { }
