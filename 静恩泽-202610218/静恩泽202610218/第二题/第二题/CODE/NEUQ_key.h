#ifndef NEUQ_KEY_H_
#define NEUQ_KEY_H_
#include "NEUQ_lib_include.h"
#define KEY_UP      PE10
#define KEY_DOWN    PE8
#define KEY_FORWARD PE7
#define KEY_BACK    PE9
#define KEY_NUM 4
#define KEY_SCAN_PERIOD_MS 10  // 按键扫描周期，单位：ms
#define KEY_SHORT_CNT 2 //短按20ms
#define KEY_LONG_CNT 50
typedef enum
{
    KEY_ACT_NONE = 0,
    KEY_ACT_SHORT,
    KEY_ACT_LONG,
} KEY_ACT_ENUM;
typedef enum{
    KEY_EVENT_NONE = 0,
    KEY_UP_SHORT,
    KEY_DOWN_SHORT,
    KEY_FORWARD_SHORT,
    KEY_BACK_SHORT,
    KEY_UP_LONG,
    KEY_DOWN_LONG,
    KEY_FORWARD_LONG,
    KEY_BACK_LONG
} KEY_EVENT_ENUM;
void NEUQ_KEY_Init(void);
void NEUQ_KEY_Scan(void);
KEY_EVENT_ENUM NEUQ_Key_EventGet(void);
#endif