#include "NEUQ_lib_include.h"
#include <math.h>

/* ★ 外部状态量 */
extern volatile uint8_t  pid_modify_rejected;
extern volatile uint8_t  motor_running;

FRAMDEF packed_data_receive;      // 存储收到的字节包
RAWDEF  opened_data_receive;      // 存储解包后的数据
NOWDEF  now_data;                 // 存储生效值
uint8_t uart_rx_buf[UART_RX_FIFO_SIZE];
NEUQ_FIFO_t uart_rx_fifo;

uint8_t new_packet_flag = 0;      // 新包到达标志

PACKING_STATE packing_state;
uint8_t pack_id;                  // 数据在包内的位置编号
uint8_t *p_recv = (uint8_t *)&packed_data_receive;

// 完整解包后，仅更新“待生效值”，由主循环根据命令决定何时生效
void Open_Pack(const FRAMDEF *packed)
{
    NEUQ_GPIO_toggle(PA8);
    if (packed->cmd == MSC_CMD_START || packed->cmd == MSC_CMD_MODIFY_SPD) {
        opened_data_receive.target_spd = packed->target_spd;
    }

    if (packed->cmd == MSC_CMD_MODIFY_PID) {
       if (!motor_running) {   // 运行中直接丢弃，不污染 opened_data_receive
        if (!isnan(packed->kp)) opened_data_receive.kp = packed->kp;
        if (!isnan(packed->ki)) opened_data_receive.ki = packed->ki;
        if (!isnan(packed->kd)) opened_data_receive.kd = packed->kd;
    }
    }
}

// 接收、组包、校验、解包；支持分次到达和连续多帧
void ReciveAndProcess_Data(void)
{
    uint8_t temp;

    while (NEUQ_FIFO_read_byte(&uart_rx_fifo, &temp) == NEUQ_FIFO_OK)
    {
        switch (packing_state)
        {
            case RECEIVE_HEAD1:
                if (temp == HEAD1) {
                    p_recv[0] = temp;
                    packing_state = RECEIVE_HEAD2;
                }
                break;

            case RECEIVE_HEAD2:
                if (temp == HEAD2) {
                    p_recv[1] = temp;
                    pack_id = 2;
                    packing_state = RECEIVE_DATA;
                }
                else if (temp == HEAD1) {
                    // 当前字节可能是新帧头1，重新保持在等待帧头2状态
                    p_recv[0] = temp;
                    packing_state = RECEIVE_HEAD2;
                }
                else {
                    packing_state = RECEIVE_HEAD1;
                }
                break;

            case RECEIVE_DATA:
                p_recv[pack_id++] = temp;

                if (pack_id >= PACK_LEN)
                {
                    // 校验帧尾，只有完整且合法的帧才交给 Open_Pack
                    if (packed_data_receive.tail[0] == TAIL1 &&
                        packed_data_receive.tail[1] == TAIL2)
                    {
                        Open_Pack(&packed_data_receive);
                        new_packet_flag = 1;   // 通知主循环：一帧完整到达
                    }

                    packing_state = RECEIVE_HEAD1;
                    pack_id = 0;
                }
                break;

            default:
                packing_state = RECEIVE_HEAD1;
                pack_id = 0;
                break;
        }
    }
}

void NEUQ_MSU_init(void)
{
    packing_state = RECEIVE_HEAD1;
    pack_id = 0;

    // 用当前生效值初始化待生效值
    opened_data_receive.target_spd = now_data.g_target_spd;
    opened_data_receive.kp = now_data.g_kp;
    opened_data_receive.ki = now_data.g_ki;
    opened_data_receive.kd = now_data.g_kd;
}

// 向 VOFA+ 回传当前生效值：目标转速、左右实际转速、PID 参数、状态字
void Pack_Data_VOFA(const FRAMDEF_VOFA *raw)
{
    float data[7];

    data[0] = raw->target_spd;
    data[1] = raw->actual_spd_lv;
    data[2] = raw->actual_spd_rv;
    data[3] = raw->kp;
    data[4] = raw->ki;
    data[5] = raw->kd;
    data[6] = raw->status;          // ★ 第 7 通道：状态字

    NEUQ_Uart_write_data(UART_0, (uint8_t *)data, sizeof(data));

    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    NEUQ_Uart_write_data(UART_0, tail, sizeof(tail));
}