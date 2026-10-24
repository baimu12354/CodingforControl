#include "key.h"


static KEY_Typedef key_table[KEY_NUM] = {
    {KEY_UP,       0, KEY_ACT_NONE},
    {KEY_DOWN,     0, KEY_ACT_NONE},
    {KEY_FORWARD,  0, KEY_ACT_NONE},
    {KEY_BACK,     0, KEY_ACT_NONE},
};

void NEUQ_Key_Init(void)
{
    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
        NEUQ_GPIO_init(key_table[i].pin, GPIO_Mode_Input_PullUp, GPIO_High);
    }
}

void NEUQ_Key_Scan(void)
{
    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
        KEY_Typedef *k = &key_table[i];

        if (NEUQ_GPIO_read(k->pin) == GPIO_Low)
        {
            k->press_cnt++;

            if (k->press_cnt == KEY_SHORT_CNT)
            {
                k->event = KEY_ACT_SHORT;
            }
            else if (k->press_cnt == KEY_LONG_CNT)
            {
                k->event = KEY_ACT_LONG;
            }
        }
        else
        {
            k->press_cnt = 0;

        }
    }
}

KEY_EVENT_ENUM NEUQ_Key_EventGet(void)
{
    KEY_EVENT_ENUM key_event = KEY_EVENT_NONE;

    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
        if (key_table[i].event != KEY_ACT_NONE)
        {
            if (key_table[i].event == KEY_ACT_LONG)
            {
                key_event = (KEY_EVENT_ENUM)(i + 5);
            }
            else
            {
                key_event = (KEY_EVENT_ENUM)(i + 1);
            }

            key_table[i].event = KEY_ACT_NONE; 
            break;                              
        }
    }
    return key_event;
}
