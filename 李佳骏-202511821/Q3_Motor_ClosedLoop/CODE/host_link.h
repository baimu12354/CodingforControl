/* host_link.h - 通信协议接口 */

#ifndef _HOST_LINK_H_
#define _HOST_LINK_H_

#include "stm32f4xx_hal.h"

/* 帧：A5 CMD LEN payload CRC16_L CRC16_H 5A
 * CRC16 覆盖 CMD+LEN+payload（0xA001，初值 0xFFFF），负载小端 */
#define HOST_LINK_MAX_PAYLOAD   (16U)

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[HOST_LINK_MAX_PAYLOAD];
} host_frame_t;

typedef struct {
    uint32_t frames_ok;         /* 校验通过并已处理的帧数 */
    uint32_t frames_bad_crc;    /* 帧尾对但校验不过 */
    uint32_t frames_bad_form;   /* 长度非法等格式错误 */
    uint32_t fifo_overflow;     /* 接收中断里 FIFO 满而丢弃的字节数 */
} host_link_stat_t;

void     HostLink_Init(void);

/* 接收中断里调用：只入队，不做解析 */
void     HostLink_OnRxIsr(uint8_t byte);

/* 主循环里反复调用：取出一帧完整且校验通过的指令，返回 1 表示取到了 */
uint8_t  HostLink_Poll(host_frame_t *frame);

void     HostLink_GetStat(host_link_stat_t *stat);

/* 校验函数单独开放，方便上位机脚本用同一套算法对照 */
uint16_t HostLink_Crc16(const uint8_t *data, uint16_t length);

/* 从负载里按小端取出一个 float */
float    HostLink_PayloadToFloat(const uint8_t *payload);

#endif /* _HOST_LINK_H_ */
