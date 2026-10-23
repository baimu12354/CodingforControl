#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

#include <stdint.h>
#include "Motor_Config.h"

/* ==================== 帧格式 ====================
 * AA 55 CMD LEN PAYLOAD 0D 0A
 */
#define MOTOR_FRAME_HEAD_1                    0xAAU
#define MOTOR_FRAME_HEAD_2                    0x55U
#define MOTOR_FRAME_TAIL_1                    0x0DU
#define MOTOR_FRAME_TAIL_2                    0x0AU
#define MOTOR_FRAME_TIMEOUT_MS                100UL

#define MOTOR_RX_BUFFER_SIZE                  128U
#define MOTOR_MAX_PAYLOAD_SIZE                32U
#define MOTOR_MAX_RX_BYTES_PER_LOOP           64U

/* ==================== 命令编号 ==================== */
#define MOTOR_CMD_START                       0x01U
#define MOTOR_CMD_STOP                        0x02U
#define MOTOR_CMD_SET_SPEED                   0x03U
#define MOTOR_CMD_SET_PID                     0x04U
#define MOTOR_CMD_QUERY                       0x05U

#define MOTOR_PID_PARAMETER_KP                0x00U
#define MOTOR_PID_PARAMETER_KI                0x01U
#define MOTOR_PID_PARAMETER_KD                0x02U

/* ==================== 执行结果 ==================== */
#define MOTOR_RESULT_OK                       0x00U
#define MOTOR_RESULT_BAD_LENGTH               0x01U
#define MOTOR_RESULT_BUSY                     0x02U
#define MOTOR_RESULT_BAD_VALUE                0x03U
#define MOTOR_RESULT_UNKNOWN_CMD              0x04U
#define MOTOR_RESULT_BAD_PARAMETER            0x05U
#define MOTOR_RESULT_CONTROL_FAULT             0x06U

/* ==================== 解析状态 ==================== */
#define MOTOR_PARSE_WAIT_HEAD_1               0U
#define MOTOR_PARSE_WAIT_HEAD_2               1U
#define MOTOR_PARSE_WAIT_COMMAND              2U
#define MOTOR_PARSE_WAIT_LENGTH               3U
#define MOTOR_PARSE_WAIT_PAYLOAD              4U
#define MOTOR_PARSE_WAIT_TAIL_1               5U
#define MOTOR_PARSE_WAIT_TAIL_2               6U

#if (MOTOR_RX_BUFFER_SIZE == 0U) || \
    ((MOTOR_RX_BUFFER_SIZE & (MOTOR_RX_BUFFER_SIZE - 1U)) != 0U) || \
    (MOTOR_RX_BUFFER_SIZE > 256U)
#error "MOTOR_RX_BUFFER_SIZE must be a power of two not greater than 256."
#endif

/* 补齐当前 NEUQ_Uart.h 中缺失、但 NEUQ_Uart.c 已实现的函数声明。 */
void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

void Motor_Protocol_Init(void);

/* UART 接收中断只调用该函数入队，不在中断中解析完整帧。 */
void Motor_Protocol_RxByte(uint8_t data);

/* 主循环调用：解析缓冲区中的分段帧或连续多帧。 */
void Motor_Protocol_ProcessRx(void);

uint8_t Motor_Protocol_StatusPending(void);
void Motor_Protocol_SendVofaStatus(void);

#endif /* MOTOR_PROTOCOL_H */
