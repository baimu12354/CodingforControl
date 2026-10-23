#ifndef _KEY_H_
#define _KEY_H_

#include "NEUQ_lib_include.h"

typedef enum{
    KEY_NONE = 0,   // 没有事件
    KEY_UP,         // 上
    KEY_DOWN,       // 下
    KEY_FORWARD,    // 进入 / 确认
    KEY_BACK,       // 返回
}KeyEvent;

void Key_Init(void);
void Key_Scan(void);        // 中断里每 2ms 调一次
KeyEvent Key_Get(void);

#endif