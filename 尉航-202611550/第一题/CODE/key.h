#ifndef KEY_H
#define KEY_H

#include "NEUQ_lib_include.h"

#define KEY PE10

typedef enum{
    KEY_ACT_NONE = 0,
    KEY_PRESS_SHORT,
    KEY_PRESS_LONG,
}KEY_ACT_ENUM;

typedef enum{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_PRESS,
}KEY_EVENT_ENUM;

void KEY_Init(void);
void KEY_Scan(void);
KEY_EVENT_ENUM KEY_Get(void);

#endif