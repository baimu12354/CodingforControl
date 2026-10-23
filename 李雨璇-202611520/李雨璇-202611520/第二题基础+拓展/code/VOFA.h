#ifndef VOFA_H_
#define VOFA_H_

#include "NEUQ_lib_include.h"

//接收陀螺仪数据
void MPU_Receive(void);
//将数据发送给VOFA+
void VOFA_send(void);
// 判断是否需要发送
uint8_t Send_Need(float a);

#endif