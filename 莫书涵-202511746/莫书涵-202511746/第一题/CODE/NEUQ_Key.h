#ifndef _NEUQ_KEY_H_
#define _NEUQ_KEY_H_

#include "NEUQ_lib_include.h"

#define KEY_UP       PE9           //key 1
#define KEY_DOWN     PE7           //key 2
#define KEY_FORWORD  PE8           //key 3
#define KEY_BACK     PE10          //key 4

#define KEY_NUM                4   //按键数量
#define KEY_SCAN_PERIOD_MS     10  //按键检测周期10ms
#define KEY_SHORT_CNT          2   //短按 20ms
#define KEY_LONG_CNT           50  //长按 500ms

typedef enum{
    KEY_ACT_NONE = 0,    //无事件
    KEY_ACT_SHORT,       //短按
    KEY_ACT_LONG,        //长按
}KEY_ACT_ENUM;

typedef enum{
    KEY_EVENT_NONE = 0,
    KEY_UP_SHORT,
    KEY_DOWN_SHORT,
    KEY_FORWORD_SHORT,
    KEY_BACK_SHORT,
    KEY_UP_LONG,
    KEY_DOWN_LONG,
    KEY_FORWORD_LONG,
    KEY_BACK_LONG,
}KEY_EVENT_ENUM;

void NEUQ_Key_Init(void);
void NEUQ_Key_Scan(void);
KEY_EVENT_ENUM NEUQ_Key_EventGet(void);

#endif
