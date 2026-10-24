/* vofa.c - VOFA+ JustFloat 发送 */

#include "vofa.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "board_config.h"

/* 回传同时从 UART0 和 UART3 发出，接哪一路都能看 */
static void Vofa_WriteByte(uint8_t byte)
{
    NEUQ_Uart_write_byte(UART_0, byte);
    NEUQ_Uart_write_byte(T3_HOST_UART, byte);
}

void Vofa_SendFloats(const float *values, uint8_t count)
{
    const uint8_t *raw;
    uint8_t        i;
    uint8_t        b;

    if (values == 0)
    {
        return;
    }

    for (i = 0; i < count; i++)
    {
        raw = (const uint8_t *)&values[i];
        for (b = 0; b < 4U; b++)
        {
            Vofa_WriteByte(raw[b]);
        }
    }

    /* JustFloat 固定帧尾 */
    Vofa_WriteByte(0x00);
    Vofa_WriteByte(0x00);
    Vofa_WriteByte(0x80);
    Vofa_WriteByte(0x7F);
}
