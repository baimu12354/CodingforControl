#ifndef _VOFA_H_
#define _VOFA_H_

#include "NEUQ_lib_include.h"

extern int32_t send_axis;   // 要发送的轴 0~5（陀螺 XYZ、加速度 XYZ）
extern int32_t send_on;     // 0 停止发送，1 发送

void Vofa_Tick(void);       // 中断里调用：每 10ms 调整一次标志
void Vofa_Task(void);       // 主循环里调用：真正发一帧

#endif