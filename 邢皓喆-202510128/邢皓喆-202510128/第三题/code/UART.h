#ifndef _UART_H_
#define _UART_H_

#include "NEUQ_lib_include.h"

extern NEUQ_FIFO_t rx_fifo;
extern uint8_t rx_buffer[128];
uint32_t TIMER_GET_MS(void);
extern volatile uint32_t timer_ms;
extern volatile uint32_t last_send;
void VOFA_SENT(void);
#endif
