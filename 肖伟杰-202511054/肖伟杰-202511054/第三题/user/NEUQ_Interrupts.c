#include "NEUQ_lib_include.h"
#include "NEUQ_MotorApp.h"

/* 第三题不用 PIT：控制节拍在 TIM5 中断里 */
void NEUQ_PIT_1_CALLBACK(void) { }
void NEUQ_PIT_2_CALLBACK(void) { }

/* USART1 接收中断：只把字节入队，解析在主循环 */
void TS_UART_0_RX_CALLBACK(uint8_t dat)
{
    NEUQ_MotorApp_RxByte(dat);
}

void TS_UART_1_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_2_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_3_RX_CALLBACK(uint8_t dat) { (void)dat; }
