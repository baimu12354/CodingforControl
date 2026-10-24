/* bsp_key.c - 按键扫描与计数消抖 */

#include "bsp_key.h"
#include "board_config.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

/* 每个按键一行：引脚、按下累计次数、待取走的事件 */
typedef struct {
    GPIO_Ports_ENUM pin;
    uint16_t        press_cnt;
    key_event_t     event;
} key_item_t;

static key_item_t s_key_table[KEY_ID_COUNT] = {
    { T1_KEY_UP_PIN,   0, KEY_EVENT_NONE },
    { T1_KEY_DOWN_PIN, 0, KEY_EVENT_NONE },
};

void Key_Init(void)
{
    uint8_t i;

    for (i = 0; i < KEY_ID_COUNT; i++)
    {
        NEUQ_GPIO_init(s_key_table[i].pin, GPIO_Mode_Input_PullUp, GPIO_High);
        s_key_table[i].press_cnt = 0;
        s_key_table[i].event     = KEY_EVENT_NONE;
    }
}

/* 10 ms 扫描一次，计数消抖 */
void Key_Scan10ms(void)
{
    uint8_t i;

    for (i = 0; i < KEY_ID_COUNT; i++)
    {
        key_item_t *key = &s_key_table[i];

        if (NEUQ_GPIO_read(key->pin) == GPIO_Low)
        {
            /* 按下：累计次数；达到长按门槛后每次扫描都补发一条长按事件 */
            if (key->press_cnt < 0xFFFFU)
            {
                key->press_cnt++;
            }
            if (key->press_cnt >= T1_KEY_LONG_TICKS)
            {
                key->event = KEY_EVENT_LONG;
            }
        }
        else
        {
            /* 松开：只有「按够消抖时间、又没到长按门槛」才算一次短按 */
            if (key->press_cnt >= T1_KEY_SHORT_MIN_TICKS &&
                key->press_cnt <  T1_KEY_LONG_TICKS)
            {
                key->event = KEY_EVENT_SHORT;
            }
            key->press_cnt = 0;
        }
    }
}

key_event_t Key_GetEvent(key_id_t id)
{
    key_event_t event;

    if (id >= KEY_ID_COUNT)
    {
        return KEY_EVENT_NONE;
    }

    event = s_key_table[id].event;
    s_key_table[id].event = KEY_EVENT_NONE;     /* 读后清空，避免同一次按下被重复计数 */
    return event;
}
