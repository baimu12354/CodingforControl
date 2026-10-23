#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "NEUQ_lib_include.h"

void Ctrl_Tick(void);           // 1ms 中断里调用（内部 5 分频，5ms 控制周期）

#endif