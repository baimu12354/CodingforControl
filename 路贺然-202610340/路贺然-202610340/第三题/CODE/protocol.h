#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "NEUQ_lib_include.h"



//初始化
void Protocol_Init(void);

//中断回调
void Protocol_RxByte(uint8_t byte);

//主循环里周期性调用 回传
void Protocol_SendFeedback(void);

void NEUQ_Work_tick(void);

#endif
