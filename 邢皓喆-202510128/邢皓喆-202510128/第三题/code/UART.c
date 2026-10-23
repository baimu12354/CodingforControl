#include "NEUQ_lib_include.h"
NEUQ_FIFO_t rx_fifo;
uint8_t rx_buffer[128];
volatile uint32_t timer_ms = 0;
volatile uint32_t last_send = 0;
uint32_t TIMER_GET_MS(void)
{
    return timer_ms;
}
void VOFA_SENT(void)
{
    static float values[5];
    values[0] = target;
    values[1] = speed_pid.Kp;
    values[2] = speed_pid.Ki;
    values[3] = speed_pid.Kd;
    values[4] = actural;
    uint8_t tail[4] = {
        0x00, 0x00, 0x80, 0x7F
    };
    uint8_t *p = (uint8_t *)values;
    for (uint32_t i = 0; i < sizeof(values); i++) {
        NEUQ_Uart_write_byte(UART_0, p[i]);
    }

    for (uint32_t i = 0; i < sizeof(tail); i++) {
        NEUQ_Uart_write_byte(UART_0, tail[i]);
    }
}
