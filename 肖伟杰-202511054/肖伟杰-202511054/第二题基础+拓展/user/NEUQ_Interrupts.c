#include "NEUQ_lib_include.h"
#include "NEUQ_Key.h"
#include "NEUQ_IMU.h"

/* 第二题：TIM6 1 ms 扫按键；TIM7 5 ms 采集 + 滤波 + 姿态 */
void NEUQ_PIT_1_CALLBACK(void)
{
    static uint8_t key_div = 0U;

    key_div++;
    if (key_div >= KEY_SCAN_DIV)
    {
        key_div = 0U;
        NEUQ_Key_Scan();
    }
}

void NEUQ_PIT_2_CALLBACK(void)
{
    NEUQ_IMU_Tick();
}

void TS_UART_0_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_1_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_2_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_3_RX_CALLBACK(uint8_t dat) { (void)dat; }
