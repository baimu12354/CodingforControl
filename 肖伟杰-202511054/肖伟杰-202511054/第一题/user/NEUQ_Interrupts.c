#include "NEUQ_lib_include.h"
#include "NEUQ_Key.h"
#include "NEUQ_LedPlan.h"

/* 第一题：TIM6 1 ms 节拍 */
void NEUQ_PIT_1_CALLBACK(void)
{
    static uint8_t key_div = 0U;

    NEUQ_LedPlan_Tick(HAL_GetTick());

    key_div++;
    if (key_div >= KEY_SCAN_DIV)
    {
        key_div = 0U;
        NEUQ_Key_Scan();
    }
}

void NEUQ_PIT_2_CALLBACK(void)
{
    /* 第一题不用 5 ms 节拍 */
}

void TS_UART_0_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_1_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_2_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_3_RX_CALLBACK(uint8_t dat) { (void)dat; }
