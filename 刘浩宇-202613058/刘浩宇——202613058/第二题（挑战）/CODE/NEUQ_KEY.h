#ifndef _NEUQ_KEY_H_
#define _NEUQ_KEY_H_

#include "NEUQ_lib_include.h"

#define KEY_UP              PE10
#define KEY_DOWN            PE8
#define KEY_FORWARD         PE7
#define KEY_BACK            PE9

#define KEY_NUM             4//按键数
#define KEY_SCSAN_PERIOD_MS 10//按键检测周期10ms
#define KEY_SHORT_CNT       2
#define KEY_LONG_CNT        50

typedef enum {//间接按键事件
    KEY_ACT_NONE=0,
    KEY_ACT_SHORT,
    KEY_ACT_LONG,
} KEY_ACT_ENUM;

typedef enum{//具体按键事件
    KEY_EVENT_NONE = 0,
    KEY_UP_SHORT,
    KEY_DOWN_SHORT,
    KEY_FORWARD_SHORT,
    KEY_BACK_SHORT,
    KEY_UP_LONG,
    KEY_DOWN_LONG,
    KEY_FORWARD_LONG,
    KEY_BACK_LONG,
}KEY_EVENT_ENUM;

void NEUQ_KEY_INIT(void);//按键初始化
void NEUQ_KEY_scan(void);//10ms一次检测按钮
KEY_EVENT_ENUM NEUQ_KEY_EVENTGET(void);//获取按键事件（将间接按键事件转化为具体按键事件）
#endif