/*
 * 基于 NEUQ 开源库（GPLv3）。
 * Copyright (c) 2025 NEUQ SmartCar Lab
 * 许可证：https://www.gnu.org/licenses/gpl-3.0.html
 */

/* 第三题：串口调参与电机闭环，独立固件。 */

#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "../CODE/host_link.h"
#include "../CODE/task_motor.h"

void NEUQ_PIT_1_CALLBACK(void)
{
    /* 第三题只用 PIT_CH2，这里保持空实现 */
}

/* TIM7 10 ms：执行一次电机闭环 */
void NEUQ_PIT_2_CALLBACK(void)
{
    Task_Motor_ControlTick();
}

/* 两路串口接收都只入 FIFO */
void TS_UART_0_RX_CALLBACK(uint8_t dat)
{
    HostLink_OnRxIsr(dat);
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
    HostLink_OnRxIsr(dat);
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
