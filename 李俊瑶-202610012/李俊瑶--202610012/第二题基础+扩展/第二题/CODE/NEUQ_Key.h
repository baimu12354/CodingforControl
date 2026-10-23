#ifndef _NEUQ_KEY_H_
#define _NEUQ_KEY_H_

#include <stdint.h>

#define KEY_NUM 4
#define KEY_SCAN_PERIOD_MS 10

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_UP_SHORT,
    KEY_DOWN_SHORT,
    KEY_FORWARD_SHORT,
    KEY_BACK_SHORT
} KeyEventEnum;

void NEUQ_Key_Init(void);
void NEUQ_Key_Scan(void);
KeyEventEnum NEUQ_Key_EventGet(void);

#endif
