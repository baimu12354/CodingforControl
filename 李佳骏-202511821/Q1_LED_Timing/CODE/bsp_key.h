/* bsp_key.h - 按键接口 */

#ifndef _BSP_KEY_H_
#define _BSP_KEY_H_

#include "stm32f4xx_hal.h"

/* 按键编号 */
typedef enum {
    KEY_ID_UP = 0,      /* PE9：上一套 */
    KEY_ID_DOWN,        /* PE7：下一套 */
    KEY_ID_COUNT
} key_id_t;

/* 按键事件。一个事件槽同一时刻只保存一条，读取后自动清空。 */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT,
    KEY_EVENT_LONG
} key_event_t;

void        Key_Init(void);
void        Key_Scan10ms(void);                     /* 每 10 ms 调用一次 */
key_event_t Key_GetEvent(key_id_t id);              /* 取走事件，取完即清空 */

#endif /* _BSP_KEY_H_ */
