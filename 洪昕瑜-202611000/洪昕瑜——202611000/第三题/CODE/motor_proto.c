/* motor_proto — 串口下行命令解析 */

#include "motor_proto.h"   /* app_common.h → 库 + config.h */
#include "motor_hw.h"      /* Motor_HW_SetPwm */
#include "motor_pid.h"     /* PID_Reset */
#include <string.h>

/* 库头文件 NEUQ_Uart.h 漏了此函数的原型(实现已在 NEUQ_Uart.c 中), 这里补声明 */
extern void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

/* 接收 FIFO(中断写, 主循环读) */
static uint8_t rx_fifo_buf[256];
static NEUQ_FIFO_t rx_fifo;

/* 接收状态机 */
typedef enum { S_HEAD1, S_HEAD2, S_CMD, S_LEN, S_DATA, S_SUM } RxState_t;
static RxState_t rx_state = S_HEAD1;
static uint8_t rx_cmd, rx_len, rx_cnt;
static uint8_t rx_buf[FRAME_DATA_MAX];

/* 把 4 字节(小端)转成 float */
static float Bytes_To_Float(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);          // STM32 为小端, 直接拷贝
    return f;
}

/* 命令派发 */
static void Proto_Dispatch(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    switch (cmd)
    {
        case CMD_START:                 /* 启动 */
            PID_Reset(&pid);
            running = 1;
            break;
        case CMD_STOP:                  /* 停止 */
            running = 0;
            Motor_HW_SetPwm(0);
            break;
        case CMD_SPEED:                 /* 目标转速 */
            if (len == 4)
            {
                target_rpm = Bytes_To_Float(data);
                // 按速度自动启停: 非零→启动, 拖到0→停止
                if (target_rpm == 0.0f) { running = 0; Motor_HW_SetPwm(0); PID_Reset(&pid); }
                else if (!running)      { PID_Reset(&pid); running = 1; }   // 首次给非零速度时启动
            }
            break;
        case CMD_KP: case CMD_KI: case CMD_KD:   /* Kp/Ki/Kd 单参数 */
            if (len == 4)
            {
                float *p = (cmd == CMD_KP) ? &pid.Kp : (cmd == CMD_KI) ? &pid.Ki : &pid.Kd;
                *p = Bytes_To_Float(data);
                if (!running) PID_Reset(&pid);
            }
            break;
        default:
            break;
    }
}

/* 字节级状态机 */
static void Proto_Feed(uint8_t b)
{
    switch (rx_state)
    {
        case S_HEAD1:
            if (b == FRAME_HEAD1) rx_state = S_HEAD2;
            break;
        case S_HEAD2:
            if (b == FRAME_HEAD2) rx_state = S_CMD;
            else if (b != FRAME_HEAD1) rx_state = S_HEAD1;
            break;
        case S_CMD:
            rx_cmd = b; rx_state = S_LEN;
            break;
        case S_LEN:
            rx_len = b;
            if (rx_len > FRAME_DATA_MAX) { rx_state = S_HEAD1; break; }
            rx_cnt = 0;
            rx_state = (rx_len == 0) ? S_SUM : S_DATA;
            break;
        case S_DATA:
            rx_buf[rx_cnt++] = b;
            if (rx_cnt >= rx_len) rx_state = S_SUM;
            break;
        case S_SUM:
            // 忽略校验和: VOFA+ 滑块的 %% 只能插入浮点小端字节, 无法动态计算 SUM, 故直接派发
            Proto_Dispatch(rx_cmd, rx_buf, rx_len);
            rx_state = S_HEAD1;
            break;
    }
}

/* 初始化通信串口 + 接收中断 + 接收 FIFO */
void Proto_Init(void)
{
    NEUQ_Uart_init(COMM_UART, COMM_BAUD);
    NEUQ_Uart_Enable_RX_Interrupt(COMM_UART, 5);
    NEUQ_FIFO_init(&rx_fifo, rx_fifo_buf, sizeof(rx_fifo_buf));
}

/* 串口接收字节入口(写入 FIFO) */
void Proto_RxByte(uint8_t dat)
{
    NEUQ_FIFO_write_byte(&rx_fifo, dat);
}

/* 主循环轮询: 从 FIFO 取字节送状态机解析 */
void Proto_Poll(void)
{
    uint8_t b;
    while (NEUQ_FIFO_read_byte(&rx_fifo, &b) == NEUQ_FIFO_OK)
    {
        Proto_Feed(b);
    }
}
