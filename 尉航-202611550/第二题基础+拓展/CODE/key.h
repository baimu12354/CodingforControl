#ifndef KEY_H
#define KEY_H

#include "NEUQ_lib_include.h"

#define KEY_UP PE10
#define KEY_DOWN PE8
#define KEY_FORWARD PE7
#define KEY_BACK PE9

#define KEY_NUM 4           //按键数量
#define KEY_SCAN_PERIOD_MS 10 //每10ms扫描一次按键状态
#define KEY_SHORT_CNT 2  //短按20ms
#define KEY_LONG_CNT 50  //长按500ms

typedef enum{
    KEY_ACT_NONE = 0,   //无事件
    KEY_ACT_SHORT,      //短按
    KEY_ACT_LONG        //长按
} KEY_ACT_ENUM;

typedef enum{
    KEY_EVENT_NONE = 0, //无事件
    KEY_UP_SHORT,
    KEY_DOWN_SHORT,
    KEY_FORWARD_SHORT,
    KEY_BACK_SHORT,
    KEY_UP_LONG,
    KEY_DOWN_LONG,
    KEY_FORWARD_LONG,
    KEY_BACK_LONG,
}  KEY_EVENT_ENUM;


//按键初始化
void Key_Init(void);
void Key_Scan(void);
KEY_EVENT_ENUM Key_EventGet(void);



#endif