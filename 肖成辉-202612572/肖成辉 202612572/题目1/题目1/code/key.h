#ifndef _KEY_H_
#define _KEY_H_
#include "NEUQ_lib_include.h"
#define KEY PE9
typedef enum{
    KEY_ACT_NONE = 0,
    KEY_PRESS_SHORT,
    KEY_PRESS_LONG,
}KEY_ACT_ENUM;
typedef enum{
    KEY_NONE,
    KEY_PRESS,
}KEY_EVENT_ENUM;
void KEY_Init(void);
void KEY_Scan(void);
KEY_EVENT_ENUM KEY_Get(void);
#endif