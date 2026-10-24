#ifndef _NEUQ_KEY_H_
#define _NEUQ_KEY_H_

#include "NEUQ_lib_include.h"

#define KEY_UP      PE10
#define KEY_DOWN    PE8
#define KEY_FORWARD PE7
#define KEY_BACK    PE9


#define KEY_NUM             4
#define KEY_SCAN_TIME_MS    10
#define KEY_SHORT_CNT       2
#define KEY_LONG_CNT        50

typedef enum{
    KEY_ACT_NONE=0,
    KEY_ACT_SHORT,
    KEY_ACT_LONG
}KEY_ACT_ENUM;


typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_UP_SHORT = 1,
    KEY_DOWN_SHORT = 2,
    KEY_FORWARD_SHORT = 3,
    KEY_BACK_SHORT = 4,
    KEY_UP_LONG = 5,
    KEY_DOWN_LONG = 6,
    KEY_FORWARD_LONG = 7,
    KEY_BACK_LONG = 8
} KEY_EVENT_ENUM;

extern KEY_EVENT_ENUM key_event;

void NEUQ_KEY_init(void);
void NEUQ_KEY_scan(void);
KEY_EVENT_ENUM NEUQ_KEY_EVENTGET(void);
#endif