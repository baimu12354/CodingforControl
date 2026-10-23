/*********************************************************************************************************************
* 文件名称          NEUQ_MotorProto
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第三题：下行字节包帧格式 + 长度驱动流式解析（纯逻辑，不碰硬件）
*
* 帧格式（自定义，无 CRC）：
*   偏移  字段     长度  说明
*   0     SOF0     1     0xAA
*   1     SOF1     1     0x55
*   2     SEQ      1     帧序号，回显在 ch8
*   3     CMD      1     命令码
*   4     LEN      1     载荷长度 0..32
*   5..   PAYLOAD  LEN   小端；float 为 IEEE754 小端 4 字节
*   整帧 = 5 + LEN 字节。
*
* 命令：
*   0x01 START      载荷 0 或 4   启动；带 4 字节 float 时同时设目标转速
*   0x02 STOP       载荷 0        停止，目标转速保留
*   0x03 SET_SPEED  载荷 4        目标转速 RPM
*   0x04 SET_PID    载荷 12       Kp,Ki,Kd
*   0x0A/0x0B/0x0C  SET_KP/KI/KD 载荷 4，单参数调 PID（VOFA 滑块用）
*
* 解析器逐字节喂入：分次到达会一直等，连续多帧一次能全部解出；只有整帧长度凑齐才交付。
********************************************************************************************************************/
#ifndef _NEUQ_MOTOR_PROTO_H_
#define _NEUQ_MOTOR_PROTO_H_

#include <stdint.h>

#define NEUQ_MOTOR_PROTO_SOF0              0xAAu
#define NEUQ_MOTOR_PROTO_SOF1              0x55u
#define NEUQ_MOTOR_PROTO_HEAD_LEN          5u
#define NEUQ_MOTOR_PROTO_CRC_LEN           0u
#define NEUQ_MOTOR_PROTO_MAX_PAYLOAD       32u
#define NEUQ_MOTOR_PROTO_MAX_FRAME         (NEUQ_MOTOR_PROTO_HEAD_LEN + NEUQ_MOTOR_PROTO_MAX_PAYLOAD)
#define NEUQ_MOTOR_PROTO_HALF_FRAME_TIMEOUT_MS   200u

#define NEUQ_MOTOR_CMD_START               0x01u
#define NEUQ_MOTOR_CMD_STOP                0x02u
#define NEUQ_MOTOR_CMD_SET_SPEED           0x03u
#define NEUQ_MOTOR_CMD_SET_PID             0x04u
#define NEUQ_MOTOR_CMD_SET_KP              0x0Au
#define NEUQ_MOTOR_CMD_SET_KI              0x0Bu
#define NEUQ_MOTOR_CMD_SET_KD              0x0Cu

typedef struct
{
    uint8_t  buf[NEUQ_MOTOR_PROTO_MAX_FRAME];   /* 正在凑的这一帧 */
    uint16_t idx;
    uint16_t need;
    uint32_t t_last;                            /* 最后一个字节的时刻 */
} NEUQ_MotorProtoType;

typedef struct
{
    uint8_t  seq;
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  payload[NEUQ_MOTOR_PROTO_MAX_PAYLOAD];
} NEUQ_MotorProtoFrameType;

void    NEUQ_MotorProto_Init(NEUQ_MotorProtoType *p, uint32_t now_ms);
uint8_t NEUQ_MotorProto_Feed(NEUQ_MotorProtoType *p, uint8_t byte, uint32_t now_ms, NEUQ_MotorProtoFrameType *out);
void    NEUQ_MotorProto_Stall(NEUQ_MotorProtoType *p, uint32_t now_ms);
float   NEUQ_MotorProto_GetF32(const uint8_t *p);

#endif /* _NEUQ_MOTOR_PROTO_H_ */
