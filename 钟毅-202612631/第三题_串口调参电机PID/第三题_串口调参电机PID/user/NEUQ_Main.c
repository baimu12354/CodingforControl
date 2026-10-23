#include "../code/NEUQ_lib/NEUQ_lib_include.h"
#include <string.h>
#include <stdio.h>

extern void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

#define UPLOAD_PERIOD_MS   10u
#define VOFA_CH_NUM        6u

static uint8_t g_rx_fifo_mem[256];
NEUQ_FIFO_t    g_rx_fifo;

static volatile uint8_t g_need_report = 0;

typedef enum
{
    ST_HEAD1 = 0,
    ST_HEAD2,
    ST_LEN,
    ST_CMD,
    ST_DATA,
    ST_SUM,
    ST_TAIL1,
    ST_TAIL2,
} ParseState_e;

static ParseState_e s_parse_st = ST_HEAD1;
static uint8_t      s_frame_len, s_frame_cmd, s_frame_sum, s_data_idx;
static uint8_t      s_data_buf[4];

static uint8_t Frame_Parse_Byte(uint8_t b)
{
    switch (s_parse_st)
    {
        case ST_HEAD1:
            if (b == 0xAAu) s_parse_st = ST_HEAD2;
            break;
        case ST_HEAD2:
            s_parse_st = (b == 0x55u) ? ST_LEN : ST_HEAD1;
            break;
        case ST_LEN:
            s_frame_len = b;
            s_frame_sum = b;
            s_data_idx  = 0;
            s_parse_st  = ST_CMD;
            break;
        case ST_CMD:
            s_frame_cmd = b;
            s_frame_sum += b;
            s_parse_st  = ST_DATA;
            break;
        case ST_DATA:
            s_data_buf[s_data_idx++] = b;
            s_frame_sum += b;
            if (s_data_idx >= 4u) s_parse_st = ST_SUM;
            break;
        case ST_SUM:
            if (b == (uint8_t)(s_frame_sum & 0xFFu))
            {
                s_parse_st = ST_TAIL1;
            }
            else
            {
                s_parse_st = ST_HEAD1;
            }
            break;
        case ST_TAIL1:
            s_parse_st = (b == 0x0Du) ? ST_TAIL2 : ST_HEAD1;
            break;
        case ST_TAIL2:
            s_parse_st = ST_HEAD1;
            return (b == 0x0Au) ? 1u : 0u;
        default:
            s_parse_st = ST_HEAD1;
            break;
    }
    return 0;
}

static float Bytes_To_Float(const uint8_t *p)
{
    float v;
    memcpy(&v, p, 4);
    return v;
}

static int32_t Bytes_To_Int(const uint8_t *p)
{
    int32_t v;
    memcpy(&v, p, 4);
    return v;
}

static void Frame_Execute(void)
{
    switch (s_frame_cmd)
    {
        case 0x01:
            Ctrl_SetTarget(Bytes_To_Float(s_data_buf));
            break;
        case 0x02: Ctrl_SetKp(Bytes_To_Float(s_data_buf)); break;
        case 0x03: Ctrl_SetKi(Bytes_To_Float(s_data_buf)); break;
        case 0x04: Ctrl_SetKd(Bytes_To_Float(s_data_buf)); break;
        case 0x05:
            Ctrl_SetRun(Bytes_To_Int(s_data_buf) ? 1u : 0u);
            break;
        case 0x06:
            g_need_report = 1;
            break;
        default: break;
    }
}

static void Comm_Task(void)
{
    uint8_t b;
    while (NEUQ_FIFO_read_byte(&g_rx_fifo, &b) == NEUQ_FIFO_OK)
    {
        if (Frame_Parse_Byte(b)) Frame_Execute();
    }
}

static void Vofa_Send_Curves(void)
{
    Ctrl_Snapshot_t s;
    uint8_t buf[VOFA_CH_NUM * 4u + 4u];
    float ch[VOFA_CH_NUM];

    Ctrl_GetSnapshot(&s);
    ch[0] = s.target_rpm;
    ch[1] = s.actual_rpm;
    ch[2] = s.pwm_out;
    ch[3] = s.kp;
    ch[4] = s.ki;
    ch[5] = s.kd;
    memcpy(buf, ch, sizeof(ch));
    buf[24] = 0x00u; buf[25] = 0x00u;
    buf[26] = 0x80u; buf[27] = 0x7Fu;
    NEUQ_Uart_write_data(UART_0, buf, (uint32_t)sizeof(buf));
}

void User_Main(void)
{
    uint32_t last_upload;
    uint32_t now;

    NEUQ_Debug_init();
    NEUQ_Delay_init();

    NEUQ_FIFO_init(&g_rx_fifo, g_rx_fifo_mem, sizeof(g_rx_fifo_mem));

    Ctrl_Init();

    NEUQ_PIT_init_ms(PIT_CH1, CTRL_PERIOD_MS);

    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 5);

    printf("Task3: motor PID closed-loop ready, waiting start command.\r\n");

    last_upload = HAL_GetTick();

    while (1)
    {
        Comm_Task();

        if (g_need_report)
        {
            g_need_report = 0;
            Vofa_Send_Curves();
        }
        now = HAL_GetTick();
        if (now - last_upload >= UPLOAD_PERIOD_MS)
        {
            last_upload = now;
            Vofa_Send_Curves();
        }
    }
}
