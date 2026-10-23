#ifndef __KEY_H__
#define __KEY_H__

#include "NEUQ_lib_include.h"

// 按键引脚：上拉输入，按下为低电平
#define KEY_UP        PE10
#define KEY_DOWN      PE8
#define KEY_FORWARD   PE7
#define KEY_ENTER     KEY_FORWARD   // 确认键，与 FORWARD 同一个脚
#define KEY_BACK      PE9
#define KEY_NONE      0

void     key_init(void);
uint8_t  key_scan(void);

#endif
