#ifndef __KEY_H__
#define __KEY_H__

#include"NEUQ_lib_include.h"


#define KEY_UP PE7
#define KEY_DOWN PE9
#define KEY_FORWARD PE10
#define KEY_BACK PE8

#define KEY_NUM 4
#define KEY_SCAN_PERIOD_MS 1
#define KEY_SHORT_CNT 2
#define KEY_LONG_CNT 50


typedef enum 
{
    KEY_ACT_NULL,KEY_ACT_SHORT,KEY_ACT_LONG
}KEY_ACT_ENUM;

typedef enum 
{
    KEY_EVENT_NULL,
    KEY_UP_SHORT,
    KEY_DOWN_SHORT,
    KEY_FORWARD_SHORT,
    KEY_BACK_SHORT,
    KEY_UP_LONG,
    KEY_DOWN_LONG,
    KEY_FORWARD_LONG,
    KEY_BACK_LONG,
}KEY_EVENT_ENUM;


void key_init(void);
void key_scan(void);
KEY_EVENT_ENUM key_eventGet(void);


#endif // !__KEY_H__