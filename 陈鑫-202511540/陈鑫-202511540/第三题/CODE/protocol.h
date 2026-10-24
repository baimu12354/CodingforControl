#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

/* ==================== 帧格式定义 ==================== */
#define FRAME_HEADER_1     0xAA
#define FRAME_HEADER_2     0x55
#define FRAME_TAIL_1       0x0D
#define FRAME_TAIL_2       0x0A

/* ==================== 命令字 ==================== */
#define FRAME_CMD_START      0x01
#define FRAME_CMD_STOP       0x02
#define FRAME_CMD_SET_TARGET 0x03
#define FRAME_CMD_SET_PID    0x04   /* 一次下发 Kp/Ki/Kd 三个 float（12 字节载荷） */
#define FRAME_CMD_QUERY      0x05
#define FRAME_CMD_SET_KP     0x06   /* 只改 Kp（4 字节 float 载荷），配滑块用 */
#define FRAME_CMD_SET_KI     0x07   /* 只改 Ki（4 字节 float 载荷），配滑块用 */
#define FRAME_CMD_SET_KD     0x08   /* 只改 Kd（4 字节 float 载荷），配滑块用 */
#define FRAME_CMD_ENC_CLR    0x09   /* 编码器累计计数清零（标定 PPR 前先清） */

#define PID_ONLINE_TUNE   1     //pid停运修改开关

/* ==================== JustFloat 通道配置 ==================== */
#define JF_DEBUG_MODE   1   /* 1=调试(15ch)  0=验收(6ch) */

/* 通道含义见 frame_parse.c 的 send_status_justfloat()；调试模式 15 路，验收模式只发前 6 路。 */
#if JF_DEBUG_MODE
    #define JF_CHANNEL_NUM   15
#else
    #define JF_CHANNEL_NUM   6
#endif

#endif
