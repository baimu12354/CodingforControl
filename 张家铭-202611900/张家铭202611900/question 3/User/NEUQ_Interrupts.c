/*********************************************************************************************************************
* 文件名称          NEUQ_Interrupts
* 功能描述          库中断回调实现：PIT 定周期控制 / 串口 RX / 外部中断
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Motor.h"

/* 定周期控制：在 PIT1（TIM6）中断里跑速度环 */
void NEUQ_PIT_1_CALLBACK(void)
{
    Motor_Control_ISR();
}

/* PIT2（TIM7）本任务未用，库会调用，需保留空实现 */
void NEUQ_PIT_2_CALLBACK(void)
{
}

/* 串口单字节接收中断：只入 FIFO，解析放主循环 */
void TS_UART_0_RX_CALLBACK(uint8_t dat)
{
    Motor_OnRxByte(dat);
}

void TS_UART_1_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_2_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_3_RX_CALLBACK(uint8_t dat) { (void)dat; }

/* 外部中断回调（本任务未用，库需链接） */
void NEUQ_EXTI0_Callback(void)  {}
void NEUQ_EXTI1_Callback(void)  {}
void NEUQ_EXTI2_Callback(void)  {}
void NEUQ_EXTI3_Callback(void)  {}
void NEUQ_EXTI4_Callback(void)  {}
void NEUQ_EXTI5_Callback(void)  {}
void NEUQ_EXTI6_Callback(void)  {}
void NEUQ_EXTI7_Callback(void)  {}
void NEUQ_EXTI8_Callback(void)  {}
void NEUQ_EXTI9_Callback(void)  {}
void NEUQ_EXTI10_Callback(void) {}
void NEUQ_EXTI11_Callback(void) {}
void NEUQ_EXTI12_Callback(void) {}
void NEUQ_EXTI13_Callback(void) {}
void NEUQ_EXTI14_Callback(void) {}
void NEUQ_EXTI15_Callback(void) {}
