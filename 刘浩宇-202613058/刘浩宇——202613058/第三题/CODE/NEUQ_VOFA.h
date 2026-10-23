#ifndef _NEUQ_VOFA_H_
#define _NEUQ_VOFA_H_

#include "NEUQ_lib_include.h"

void NEUQ_VOFA_INIT(void);// 初始化接收FIFO，开启串口接收中断
void NEUQ_VOFA_RecvByte(uint8_t dat);//每收到 1 字节就丢进 FIFO
void NEUQ_VOFA_Process(void);//从 FIFO 取字节、解包、执行指令
void NEUQ_VOFA_Send(void);//每 20ms 向 VOFA+ 上传 5 通道曲线

#endif
