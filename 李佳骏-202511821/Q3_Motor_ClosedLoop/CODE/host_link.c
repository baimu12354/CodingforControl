/* host_link.c - 二进制帧解析与 CRC16 */

#include "host_link.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "board_config.h"

/* 接收缓冲区大小。115200 下每字节约 87 μs，256 字节可以吸收 22 ms 的突发，
 * 足够覆盖一次整屏操作那么长的阻塞。 */
#define HOST_LINK_FIFO_SIZE     (256U)

/* 滑块指令不带 CRC，见 board_config.h 里 T3_CMD_SET_TARGET_SLIDER 的说明 */
#define HOST_CMD_NO_CRC(cmd)    ((cmd) == T3_CMD_SET_TARGET_SLIDER)

typedef enum {
    ST_WAIT_HEAD = 0,
    ST_WAIT_CMD,
    ST_WAIT_LEN,
    ST_WAIT_PAYLOAD,
    ST_WAIT_CRC_L,
    ST_WAIT_CRC_H,
    ST_WAIT_TAIL
} host_parse_state_t;

static uint8_t            s_storage[HOST_LINK_FIFO_SIZE];
static NEUQ_FIFO_t        s_fifo;

static host_parse_state_t s_state = ST_WAIT_HEAD;
static host_frame_t       s_building;
static uint8_t            s_payload_index = 0;
static uint16_t           s_crc_recv = 0;
static host_link_stat_t   s_stat;

uint16_t HostLink_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  bit;

    for (i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i];
        for (bit = 0; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

float HostLink_PayloadToFloat(const uint8_t *payload)
{
    float value;
    uint8_t *p = (uint8_t *)&value;

    p[0] = payload[0];
    p[1] = payload[1];
    p[2] = payload[2];
    p[3] = payload[3];
    return value;
}

void HostLink_Init(void)
{
    NEUQ_FIFO_init(&s_fifo, s_storage, sizeof(s_storage));

    s_state    = ST_WAIT_HEAD;
    s_payload_index = 0;
    s_crc_recv = 0;
    s_stat.frames_ok       = 0;
    s_stat.frames_bad_crc  = 0;
    s_stat.frames_bad_form = 0;
    s_stat.fifo_overflow   = 0;

    /* UART0(PA9/10) 与 UART3(PC10/11) 都开，接收都进同一 FIFO */
    NEUQ_Uart_init(UART_0, T3_HOST_BAUD);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, T3_HOST_RX_PRIORITY);

    NEUQ_Uart_init(T3_HOST_UART, T3_HOST_BAUD);
    NEUQ_Uart_Enable_RX_Interrupt(T3_HOST_UART, T3_HOST_RX_PRIORITY);
}

void HostLink_OnRxIsr(uint8_t byte)
{
    if (NEUQ_FIFO_write_byte(&s_fifo, byte) != NEUQ_FIFO_OK)
    {
        s_stat.fifo_overflow++;
    }
}

/* 逐字节状态机，支持半帧和连帧 */
static uint8_t HostLink_HandleByte(uint8_t byte)
{
    switch (s_state)
    {
        case ST_WAIT_HEAD:
            if (byte == T3_FRAME_HEAD)
            {
                s_state = ST_WAIT_CMD;
            }
            /* 不是帧头就直接丢掉，帧头之前的杂字节会被自然忽略 */
            break;

        case ST_WAIT_CMD:
            s_building.cmd = byte;
            s_state = ST_WAIT_LEN;
            break;

        case ST_WAIT_LEN:
            if (byte > HOST_LINK_MAX_PAYLOAD)
            {
                /* 长度非法：这一帧不可救，回到找帧头 */
                s_stat.frames_bad_form++;
                s_state = ST_WAIT_HEAD;
                s_payload_index = 0;
                break;
            }
            s_building.len = byte;
            s_payload_index = 0;
            if (byte == 0U)
            {
                s_state = HOST_CMD_NO_CRC(s_building.cmd) ? ST_WAIT_TAIL : ST_WAIT_CRC_L;
            }
            else
            {
                s_state = ST_WAIT_PAYLOAD;
            }
            {
                uint8_t k;
                for (k = 0; k < HOST_LINK_MAX_PAYLOAD; k++)
                {
                    s_building.payload[k] = 0;
                }
            }
            break;

        case ST_WAIT_PAYLOAD:
            /* 记录本帧负载进度。这个计数在收到 LEN 时清零，
             * 所以即使上一帧中途断了，也不会把残留的进度带进新的一帧。 */
            s_building.payload[s_payload_index] = byte;
            s_payload_index++;
            if (s_payload_index >= s_building.len)
            {
                s_payload_index = 0;
                s_state = HOST_CMD_NO_CRC(s_building.cmd) ? ST_WAIT_TAIL : ST_WAIT_CRC_L;
            }
            break;

        case ST_WAIT_CRC_L:
            s_crc_recv = byte;
            s_state = ST_WAIT_CRC_H;
            break;

        case ST_WAIT_CRC_H:
            s_crc_recv = (uint16_t)(s_crc_recv | ((uint16_t)byte << 8));
            s_state = ST_WAIT_TAIL;
            break;

        case ST_WAIT_TAIL:
        {
            uint8_t  crc_buf[HOST_LINK_MAX_PAYLOAD + 2U];
            uint16_t crc_calc;
            uint8_t  k;
            uint8_t  need_crc = (uint8_t)(HOST_CMD_NO_CRC(s_building.cmd) ? 0U : 1U);

            crc_calc = 0;
            if (need_crc != 0U)
            {
                crc_buf[0] = s_building.cmd;
                crc_buf[1] = s_building.len;
                for (k = 0; k < s_building.len; k++)
                {
                    crc_buf[2U + k] = s_building.payload[k];
                }
                crc_calc = HostLink_Crc16(crc_buf, (uint16_t)(s_building.len + 2U));
            }

            s_state = ST_WAIT_HEAD;

            if (byte != T3_FRAME_TAIL)
            {
                s_stat.frames_bad_form++;
                break;
            }
            if ((need_crc != 0U) && (crc_calc != s_crc_recv))
            {
                s_stat.frames_bad_crc++;
                break;
            }
            return 1U;
        }

        default:
            s_state = ST_WAIT_HEAD;
            break;
    }

    return 0U;
}

/* 主循环取帧，返回 1 表示收到完整帧 */
uint8_t HostLink_Poll(host_frame_t *frame)
{
    uint8_t byte;

    while (NEUQ_FIFO_read_byte(&s_fifo, &byte) == NEUQ_FIFO_OK)
    {
        if (HostLink_HandleByte(byte) != 0U)
        {
            if (frame != 0)
            {
                *frame = s_building;
            }
            s_stat.frames_ok++;
            return 1U;
        }
    }
    return 0U;
}

void HostLink_GetStat(host_link_stat_t *stat)
{
    if (stat != 0)
    {
        *stat = s_stat;
    }
}
