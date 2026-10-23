#ifndef _NEUQ_PACK_H_
#define _NEUQ_PACK_H_
#include "NEUQ_lib_include.h"


#define FRAME_HEAD1     0xAA
#define FRAME_HEAD2     0x55
#define CMD_START       0x01    // 启动
#define CMD_STOP        0x02    // 停止
#define CMD_SET_TARGET  0x03    // 设目标转速
#define CMD_SET_PID     0x04    // 批量设 PID（3×float32）
#define CMD_SET_KP      0x05    // 设 Kp
#define CMD_SET_KI      0x06    // 设 Ki
#define CMD_SET_KD      0x07    // 设 Kd
#define FRAME_DATA_MAX  16

// 电机运行状态
extern volatile int32_t target_speed;   // 目标转速
extern volatile int32_t actual_speed;   // 实际转速
extern volatile uint8_t  motor_enable;  // 使能，0停，1转
extern pid_value motor_pid;             // PID 状态

// 串口接收 FIFO
extern NEUQ_FIFO_t uart_rx_fifo;

void NEUQ_Pack_init(void);
void protocol_parse_byte(uint8_t b);
void protocol_send_echo(void);

#endif
