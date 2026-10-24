#include "NEUQ_lib_include.h"

extern void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

#define PROTOCOL_UART      UART_0            // 调参 + 回传共用一个串口
#define RX_BUF_SIZE        64                // 接收环形缓冲大小（字节）

/* 用联合体把 float 拆成 4 个字节（小端），避免指针强转的别名问题 */
typedef union {
    float    f;
    uint8_t  b[4];
} FloatBytes;

/*------------------ 接收环形缓冲（中断写 / 主循环读） ------------------*/
static volatile uint8_t rx_buf[RX_BUF_SIZE];      // 缓冲区本体
static volatile uint8_t rx_head = 0;              // 写指针（中断里移动）
static volatile uint8_t rx_tail = 0;              // 读指针（主循环里移动）

/* 入队：中断回调调用。缓冲满时丢弃最新字节（帧很小，64字节足够） */
static void rx_buf_put(uint8_t d)
{
    uint8_t next = (uint8_t)((rx_head + 1) % RX_BUF_SIZE);
    if (next != rx_tail) {                        // 没满才写入
        rx_buf[rx_head] = d;
        rx_head = next;
    }
}

/* 出队：主循环调用。成功返回1，空缓冲返回0 */
static uint8_t rx_buf_get(uint8_t *d)
{
    if (rx_head == rx_tail) return 0;             // 空
    *d = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1) % RX_BUF_SIZE);
    return 1;
}

/*------------------ 接收状态机变量 ------------------*/
static uint8_t rx_state = RX_STATE_IDLE;          // 当前状态
static uint8_t rx_cmd   = 0;                      // 命令字
static uint8_t rx_len   = 0;                      // 数据长度
static uint8_t rx_cnt   = 0;                      // 已收数据个数
static uint8_t rx_data[PROTOCOL_DATA_MAX];        // 数据区暂存
static uint8_t rx_sum   = 0;                      // 累加校验（自动按8位截断）

/*------------------ 帧解析完成后的命令分发 ------------------*/
static void protocol_dispatch(void)
{
    switch (rx_cmd) {
        case CMD_START:
            Speed_Start();
            break;

        case CMD_STOP:
            Speed_Stop();
            break;

        case CMD_SET_TARGET: {                    // DATA 4字节 = int32_t 目标转速
            int32_t target = 0;
            memcpy(&target, rx_data, 4);          // 小端机器上直接拷贝即可
            Speed_Target(target);
            break;
        }

        case CMD_SET_PID: {                       // DATA 12字节 = 3个float
            float kp = 0.0f, ki = 0.0f, kd = 0.0f;
            memcpy(&kp, rx_data,      4);
            memcpy(&ki, rx_data + 4,  4);
            memcpy(&kd, rx_data + 8,  4);
            Speed_PID(kp, ki, kd);
            break;
        }

        default:
            break;                                // 未知命令：忽略整帧
    }
}

/*------------------ 状态机：每来一个字节推进一次 ------------------*/
static void protocol_parse_byte(uint8_t d)
{
    switch (rx_state) {
        case RX_STATE_IDLE:                       // 空闲，找帧头1
            if (d == PROTOCOL_HEAD1) rx_state = RX_STATE_HEAD1;
            break;

        case RX_STATE_HEAD1:                      // 找帧头2，失败回空闲
            rx_state = (d == PROTOCOL_HEAD2) ? RX_STATE_CMD : RX_STATE_IDLE;
            break;

        case RX_STATE_CMD:                        // 命令字
            rx_cmd = d;
            rx_state = RX_STATE_LEN;
            break;

        case RX_STATE_LEN:                        // 数据长度，顺带把"帧头+命令+长度"累加进校验
            rx_len = d;
            rx_cnt = 0;
            rx_sum = PROTOCOL_HEAD1 + PROTOCOL_HEAD2 + rx_cmd + rx_len;
            if (rx_len == 0) {
                rx_state = RX_STATE_SUM;          // 无数据，直接等校验
            } else if (rx_len > PROTOCOL_DATA_MAX) {
                rx_state = RX_STATE_IDLE;         // 长度非法，重新找帧头
            } else {
                rx_state = RX_STATE_DATA;
            }
            break;

        case RX_STATE_DATA:                       // 收数据区
            rx_data[rx_cnt++] = d;
            rx_sum += d;                          // 边收边累加校验
            if (rx_cnt >= rx_len) rx_state = RX_STATE_SUM;
            break;

        case RX_STATE_SUM:                        // 校验通过才执行命令
            if (rx_sum == d) protocol_dispatch();
            rx_state = RX_STATE_IDLE;             // 无论对错都回到空闲，可继续收下一帧
            break;

        default:
            rx_state = RX_STATE_IDLE;
            break;
    }
}

/*------------------ 初始化：配置串口并打开接收中断 ------------------*/
void Protocol_Init(void)
{
    NEUQ_Uart_init(PROTOCOL_UART, UART_BAUD_115200);
    /* 优先级5：低于PIT周期中断(0)，电机控制不会被通信长时间打断 */
    NEUQ_Uart_Enable_RX_Interrupt(PROTOCOL_UART, 5);
}

/*------------------ 串口中断回调入口：只做入队，立即返回 ------------------*/
void Protocol_RX_Byte(uint8_t dat)
{
    rx_buf_put(dat);
}

/*------------------ 主循环任务：把缓冲里的字节全部喂给状态机 ------------------*/
void Protocol_Task(void)
{
    uint8_t d;
    while (rx_buf_get(&d)) {
        protocol_parse_byte(d);
    }
}

/*------------------ VOFA+ JustFloat 回传帧 ------------------*/
void Protocol_Send_Status(void)
{
    FloatBytes fb;
    float ch[5];

    ch[0] = (float)Speed_GetTarget();
    ch[1] = (float)Speed_GetActual();
    ch[2] = Speed_GetKP();
    ch[3] = Speed_GetKI();
    ch[4] = Speed_GetKD();

    for (int i = 0; i < 5; i++) {
        fb.f = ch[i];
        for (int j = 0; j < 4; j++) {
            NEUQ_Uart_write_byte(PROTOCOL_UART, fb.b[j]);   // float 低字节在前（小端）
        }
    }

    /* JustFloat 帧尾标记 */
    NEUQ_Uart_write_byte(PROTOCOL_UART, 0x00);
    NEUQ_Uart_write_byte(PROTOCOL_UART, 0x00);
    NEUQ_Uart_write_byte(PROTOCOL_UART, 0x80);
    NEUQ_Uart_write_byte(PROTOCOL_UART, 0x7F);
}
