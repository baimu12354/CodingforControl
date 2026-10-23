#ifndef _NEUQ_KEY_H
#define _NEUQ_KEY_H

#include "NEUQ_lib_include.h"
//上下确定取消
#define KEY_UP     PE10
#define KEY_DOWN   PE8
#define KEY_DONE   PE7
#define KEY_ABORT  PE9
//按键数量，扫描周期，短按与长按

#define KEY_NUM            4
#define KEY_SCAN_PERIOD_MS 10
#define KEY_SHORT_MS       2
#define KEY_LONG_MS        50
//按键状态
typedef enum{
    KEY_ACT_NONE = 0,
    KEY_ACT_SHORT,
    KEY_ACT_LONG
} KEY_ACT_ENUM;
//按键状态
typedef enum{
    KEY_EVENT_NONE = 0,
    KEY_UP_SHORT,
    KEY_DOWN_SHORT,
    KEY_DONE_SHORT,
    KEY_ABORT_SHORT,
    KEY_UP_LONG,
    KEY_DOWN_LONG,
    KEY_DONE_LONG,
    KEY_ABORT_LONG
} KEY_EVENT_ENUM;

void NEUQ_Key_Init(void);
void NEUQ_Key_Scan(void);
KEY_EVENT_ENUM NEUQ_KEY_EventGet(void);

#endif