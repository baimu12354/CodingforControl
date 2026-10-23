#ifndef _NEUQ_KEY_H_
#define _NEUQ_KEY_H_

#include "NEUQ_lib_include.h"

#define  KEY_UP     PE10
#define KEY_DOWN    PE8
#define KEY_FORWARD PE7
#define KEY_BACK    PE9

#define KEY_NUM 4
#define KEY_SCAN_PERIOD_MS 10 //按键检测周期10ms
#define KEY_SHORT_CNT 2 //短按20ms
#define KEY_LONG_CNT 50 //长按500ms

typedef enum{
    KEY_ACT_NONE = 0, //无事件
    KEY_ACT_SHORT, //短按
    KEY_ACT_LONG //长按
}KEY_ACT_ENUM; //单个按键的枚举类型

typedef enum{
    KEY_EVENT_NONE = 0, //无事件
    KEY_UP_SHORT, //1
    KEY_DOWN_SHORT, //2
    KEY_FORWARD_SHORT, //3
    KEY_BACK_SHORT, //4
    KEY_UP_LONG, //5
    KEY_DOWN_LONG, //6
    KEY_FORWARD_LONG, //7
    KEY_BACK_LONG //8
}KEY_EVENT_ENUM; //整个系统的枚举类型

void NEUQ_Key_Init(void); //按键的初始化
void NEUQ_Key_Scan(void); ///按键的扫描
KEY_EVENT_ENUM NEUQ_Key_EventGet(void); //获取按键内部的事件

#endif 