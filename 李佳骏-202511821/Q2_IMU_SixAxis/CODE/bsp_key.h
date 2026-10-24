/* bsp_key.h - 按键接口 */

#ifndef _BSP_KEY_H_
#define _BSP_KEY_H_

#include "stm32f4xx_hal.h"

typedef enum {
    KEY_ID_UP = 0,      /* PE9  上移 */
    KEY_ID_DOWN,        /* PE7  下移 */
    KEY_ID_ENTER,       /* PE8  进入 / 确认 */
    KEY_ID_BACK,        /* PE10 返回 / 取消 */
    KEY_ID_COUNT
} key_id_t;

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT,
    KEY_EVENT_LONG
} key_event_t;

void        Key_Init(void);
void        Key_Scan10ms(void);                     /* 每 10 ms 调用一次 */
key_event_t Key_GetEvent(key_id_t id);              /* 取走事件，取完即清空 */

#endif /* _BSP_KEY_H_ */
