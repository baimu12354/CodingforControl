#ifndef _KEY_H
#define _KEY_H

#include "NEUQ_lib_include.h"

#define KEY_Port PE9
#define KEY_CNT_MIN  20

typedef enum
{
    KEY_ACT_OFF = 0,
    KEY_ACT_ON
} KEY_ActionTypeDef;

typedef struct 
{
    GPIO_Ports_ENUM Ports;
    uint8_t cnt;
}KeyConfig;

extern KeyConfig KEY;

void KEY_init(void);
void KEY_Scan(void);

#endif
