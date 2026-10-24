#include "NEUQ_lib_include.h"

#define UART_RX_FIFO_SIZE  128
uint8_t  uart_rx_fifo_buf[UART_RX_FIFO_SIZE];
NEUQ_FIFO_t     uart_rx_fifo;
static uint32_t uart_rx_overflow_cnt = 0;


void TS_UART_0_RX_CALLBACK(uint8_t dat) {
   if (NEUQ_FIFO_write_byte(&uart_rx_fifo, dat) != NEUQ_FIFO_OK) {
        uart_rx_overflow_cnt++;
    }
}

uint32_t get_uart_rx_overflow(void) {
    return uart_rx_overflow_cnt;
}

void TS_UART_1_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_2_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_3_RX_CALLBACK(uint8_t dat) { (void)dat; }

/* PIT 中断 */
void NEUQ_PIT_1_CALLBACK(void) {
    if (sys_running ) {
        motor_ctrl();              
    } else {
        motor_pid_reset();          /* 停机：清 PID 并把 duty_now 归零（否则通道 12 会一直显示上次的 -10000） */
        motor_setduty(0);
    }
}

void NEUQ_PIT_2_CALLBACK(void) {}

/* EXTI 回调全留空 */
void NEUQ_EXTI0_Callback(void)  {}
void NEUQ_EXTI1_Callback(void)  {}
void NEUQ_EXTI2_Callback(void)  {}
void NEUQ_EXTI3_Callback(void)  {}
void NEUQ_EXTI4_Callback(void)  {}
void NEUQ_EXTI9_5_Callback(void) {}
void NEUQ_EXTI15_10_Callback(void) {}
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