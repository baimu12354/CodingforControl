#ifndef _KEY_H_
#define _KEY_H_

#include "NEUQ_lib_include.h"

#define MODE1     PE10  //方案1
#define MODE2     PE8   //方案2
#define MODE3     PE7   //方案3
#define MODE4     PE9   //方案4

#define NUM  4
#define SHORT_CNT  20        //短按 20ms

typedef enum{
    KEY_ACT_NONE = 0,   //无事件
    KEY_ACT_SHORT,      //短按
} KEY_ACT_ENUM;

typedef enum{
    EVENT_NONE = 0, //无事件
    EVENT1,         //方案1
    EVENT2,         //方案2
    EVENT3,         //方案3
    EVENT4,         //方案4
} KEY_EVENT_ENUM;

void NEUQ_Key_Init(void);
void NEUQ_Key_Scan(void);
KEY_EVENT_ENUM NEUQ_Key_EventGet(void);

#endif