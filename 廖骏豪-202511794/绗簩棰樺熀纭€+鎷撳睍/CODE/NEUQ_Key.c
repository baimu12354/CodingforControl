/*
 * NEUQ_Key.c
 * 四个按键的扫描 / 消抖 / 短按长按识别
 *
 * 事件编号规律：短按 = 索引*2+1，长按 = 索引*2+2
 *   索引0 KEY_UP      -> 1 KEY_UP_SHORT      2 KEY_UP_LONG
 *   索引1 KEY_DOWN    -> 3 KEY_DOWN_SHORT    4 KEY_DOWN_LONG
 *   索引2 KEY_FORWARD -> 5 KEY_FORWARD_SHORT 6 KEY_FORWARD_LONG
 *   索引3 KEY_BACK    -> 7 KEY_BACK_SHORT    8 KEY_BACK_LONG
 */
#include "NEUQ_lib_include.h"
#include "NEUQ_Key.h"

typedef struct
{
    GPIO_Ports_ENUM pin;        // 引脚
    uint16_t press_cnt;         // 按下持续时间，单位是扫描周期
    uint16_t repeat_cnt;        // 长按重复计数
    uint8_t  long_sent;         // 这次按下有没有报过长按
    KEY_ACT_ENUM event;         // 等着被取走的动作
} KEY_Typedef;

static KEY_Typedef key_table[KEY_NUM] = {
    {KEY_UP,      0, 0, 0, KEY_ACT_NONE},
    {KEY_DOWN,    0, 0, 0, KEY_ACT_NONE},
    {KEY_FORWARD, 0, 0, 0, KEY_ACT_NONE},
    {KEY_BACK,    0, 0, 0, KEY_ACT_NONE}
};

void NEUQ_Key_Init(void)
{
    uint8_t i;
    for(i = 0; i < KEY_NUM; i++)
    {
        NEUQ_GPIO_init(key_table[i].pin, GPIO_Mode_Input_PullUp, GPIO_High);
    }
}

// 按键扫描，每10ms调一次
void NEUQ_Key_Scan(void)
{
    uint8_t i;

    for(i = 0; i < KEY_NUM; i++)
    {
        KEY_Typedef *key = &key_table[i];

        if(NEUQ_GPIO_read(key->pin) == GPIO_Low)        // 按下，低电平有效
        {
            key->press_cnt++;

            // 只有UP/DOWN做长按连发，OK/BACK按多久都只算一次短按
            if(i < KEY_LONG_REPEAT_NUM)
            {
                if(key->long_sent == 0)
                {
                    // 还没报过长按，按住到KEY_LONG_CNT就报一次
                    if(key->press_cnt >= KEY_LONG_CNT)
                    {
                        key->long_sent = 1;
                        key->repeat_cnt = 0;
                        key->event = KEY_ACT_LONG;
                    }
                }
                else
                {
                    // 已经报过长按了，之后每KEY_REPEAT_CNT重复一次
                    key->repeat_cnt++;
                    if(key->repeat_cnt >= KEY_REPEAT_CNT)
                    {
                        key->repeat_cnt = 0;
                        key->event = KEY_ACT_LONG;
                    }
                }
            }
        }
        else                                            // 松手
        {
            // 按够消抖时间就算一次短按，UP/DOWN如果已经报过长按就不再补
            if(key->press_cnt >= KEY_SHORT_CNT && key->long_sent == 0)
            {
                key->event = KEY_ACT_SHORT;
            }
            key->press_cnt  = 0;
            key->repeat_cnt = 0;
            key->long_sent  = 0;
        }
    }
}

// 取走一个待处理的按键事件，应用层用while一直取到返回KEY_EVENT_NONE
// 每调一次只返回一个并清掉它，所以同一周期里两个键同时按下也不会漏
KEY_EVENT_ENUM NEUQ_Key_GetEvent(void)
{
    KEY_EVENT_ENUM key_event = KEY_EVENT_NONE;
    uint8_t i;

    for(i = 0; i < KEY_NUM; i++)
    {
        if(key_table[i].event != KEY_ACT_NONE)
        {
            // 短按 = i*2+1，长按 = i*2+2
            key_event = (KEY_EVENT_ENUM)(i * 2 + ((key_table[i].event == KEY_ACT_LONG) ? 2 : 1));
            key_table[i].event = KEY_ACT_NONE;
            break;
        }
    }
    return key_event;
}
