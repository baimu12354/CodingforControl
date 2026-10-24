#ifndef NEUQ_MSC_H_
#define NEUQ_MSC_H_

#define HEAD1       0x55
#define HEAD2       0xAA
#define TAIL1       0x0D
#define TAIL2       0x0A

// 命令码
#define MSC_CMD_START    0x01    // 启动电机
#define MSC_CMD_STOP     0x02    // 停止电机
#define MSC_CMD_MODIFY_SPD 0x03  // 修改目标转速
#define MSC_CMD_MODIFY_PID 0x04  // 修改PID参数（仅停机时生效）

#define UART_RX_FIFO_SIZE 256
#define PACK_LEN          sizeof(FRAMDEF)

// 接收帧结构体
typedef __packed struct
{
    uint8_t  head[2];
    uint8_t  cmd;
    uint8_t  len;
    float    target_spd;
    float    kp;
    float    ki;
    float    kd;
    uint8_t  tail[2];
} FRAMDEF;

// 解包之后参数结构体
typedef struct
{
    float target_spd;
    float kp;
    float ki;
    float kd;
} RAWDEF;

// 当前正在生效的值
typedef struct
{
    float g_target_spd;
    float g_kp;
    float g_ki;
    float g_kd;
} NOWDEF;

// ★ 发送帧结构体（增加第 7 个状态字）
//   status = bit0(PID修改被拒) | bit1(电机运行中)
typedef struct
{
    float target_spd;
    float actual_spd_lv;
    float actual_spd_rv;
    float kp;
    float ki;
    float kd;
    float status;
} FRAMDEF_VOFA;

typedef enum{
    RECEIVE_HEAD1=1,
    RECEIVE_HEAD2,
    RECEIVE_DATA,
} PACKING_STATE;

extern FRAMDEF packed_data_receive;
extern RAWDEF  opened_data_receive;
extern NOWDEF  now_data;
extern uint8_t uart_rx_buf[UART_RX_FIFO_SIZE];
extern NEUQ_FIFO_t uart_rx_fifo;
extern uint8_t new_packet_flag;

void ReciveAndProcess_Data(void);
void NEUQ_MSU_init(void);
void Pack_Data_VOFA(const FRAMDEF_VOFA *raw);

#endif