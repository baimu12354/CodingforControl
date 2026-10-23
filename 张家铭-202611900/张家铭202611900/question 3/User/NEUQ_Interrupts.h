#ifndef _NEUQ_INTERRUPTS_H_
#define _NEUQ_INTERRUPTS_H_
#include "stm32f4xx_hal.h"

void NEUQ_PIT_1_CALLBACK(void);
void NEUQ_PIT_2_CALLBACK(void);

void TS_UART_0_RX_CALLBACK(uint8_t dat);
void TS_UART_1_RX_CALLBACK(uint8_t dat);
void TS_UART_2_RX_CALLBACK(uint8_t dat);
void TS_UART_3_RX_CALLBACK(uint8_t dat);

void NEUQ_EXTI0_Callback(void);
void NEUQ_EXTI1_Callback(void);
void NEUQ_EXTI2_Callback(void);
void NEUQ_EXTI3_Callback(void);
void NEUQ_EXTI4_Callback(void);
void NEUQ_EXTI5_Callback(void);
void NEUQ_EXTI6_Callback(void);
void NEUQ_EXTI7_Callback(void);
void NEUQ_EXTI8_Callback(void);
void NEUQ_EXTI9_Callback(void);
void NEUQ_EXTI10_Callback(void);
void NEUQ_EXTI11_Callback(void);
void NEUQ_EXTI12_Callback(void);
void NEUQ_EXTI13_Callback(void);
void NEUQ_EXTI14_Callback(void);
void NEUQ_EXTI15_Callback(void);

#endif
