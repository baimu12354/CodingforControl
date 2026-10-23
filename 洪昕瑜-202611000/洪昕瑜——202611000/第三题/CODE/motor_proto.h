#ifndef _MOTOR_PROTO_H_
#define _MOTOR_PROTO_H_

#include "app_common.h"

void Proto_Init(void);          /* 初始化通信串口 + 接收中断 + 接收 FIFO */
void Proto_RxByte(uint8_t dat); /* 串口接收字节入口(写入 FIFO) */
void Proto_Poll(void);          /* 主循环轮询: 解析下行帧 */

#endif /* _MOTOR_PROTO_H_ */
