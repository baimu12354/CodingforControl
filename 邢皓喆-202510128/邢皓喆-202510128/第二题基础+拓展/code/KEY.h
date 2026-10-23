#ifndef KEY_H
#define KEY_H

#include "NEUQ_lib_include.h"

#define KEY_UP PE9
#define KEY_DOWN PE8
#define KEY_FORWARD PE7
#define KEY_BEHIND PE10
#define KEY_CNT_MIN 1
#define KEY_PRESS_SHOART_CNT 9
#define KEY_PRESS_LONG_CNT 19
#define KEY_NUM 4
#define KEY_SCAN_PERIOD 10

typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_UP_SHORT = 1,
    KEY_DOWN_SHORT = 2,
    KEY_FORWARD_SHORT = 3,
    KEY_BEHIND_SHORT = 4,
    KEY_UP_LONG = 5,
    KEY_DOWN_LONG = 6,
    KEY_FORWARD_LONG = 7,
    KEY_BEHIND_LONG = 8,
} KEY_EventTypeDef;

typedef enum
{
    KEY_ACT_NONE = 0,
    KEY_ACT_SHORT,
    KEY_ACT_LONG,
} KEY_ActionTypeDef;

typedef struct
{
    GPIO_Ports_ENUM  KEY_PIN;
    uint8_t KEY_PRESS_CNT;
    volatile KEY_ActionTypeDef KEY_ACT;
} KEY_TypeDef;

void KEY_Init(void);
void KEY_Scan(void);
KEY_EventTypeDef KEY_GetEvent(void);
#endif
