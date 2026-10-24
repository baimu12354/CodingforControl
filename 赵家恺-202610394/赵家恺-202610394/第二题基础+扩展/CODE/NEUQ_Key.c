#include "NEUQ_Key.h"

// 按键表：引脚 / 按下计数 / 重复计数 / 本次事件
KEY_Typedef key_table[KEY_NUM] = {
    {KEY_UP,      0, 0, KEY_ACT_NONE},
    {KEY_DOWN,    0, 0, KEY_ACT_NONE},
    {KEY_FORWARD, 0, 0, KEY_ACT_NONE},
    {KEY_BACK,    0, 0, KEY_ACT_NONE},
};

void NEUQ_Key_Init(void)
{
    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
        // pin 存的是 int16_t，这里显式转回 GPIO_Ports_ENUM，避免类型不匹配的告警
        NEUQ_GPIO_init((GPIO_Ports_ENUM)key_table[i].pin, GPIO_Mode_Input_PullUp, GPIO_High);
    }
}

// 放在 10ms 的 PIT 中断里周期调用（见 NEUQ_Interrupts.c）
//
// 关键点：一次"按下"只产生一次短按事件，一次"长按"只产生一次长按事件。
// 只在"刚好越过阈值的那一个扫描周期"里置位，而不是按住期间每个周期都置位，
// 否则按一下 50ms，主循环就会收到 5 个短按事件，菜单光标一下跳 5 格。
void NEUQ_Key_Scan(void)
{
    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
        KEY_Typedef *k = &key_table[i];

        if (NEUQ_GPIO_read((GPIO_Ports_ENUM)k->pin) == GPIO_Low) // 按下（低电平有效）
        {
            if (k->press_cnt < KEY_LONG_CNT)
            {
                k->press_cnt++;                     // 计数封顶在长按阈值，不会溢出

                if (k->press_cnt == KEY_SHORT_CNT)
                {
                    k->event = KEY_ACT_SHORT;       // 越过短按阈值：消抖完成，只报一次
                }
                else if (k->press_cnt == KEY_LONG_CNT)
                {
                    k->event      = KEY_ACT_LONG;   // 越过长按阈值：只报一次
                    k->repeat_cnt = 0;
                }
            }
            else
            {
                // 已经进入长按：每隔 KEY_REPEAT_CNT 个周期补报一次长按事件，
                // 这样一直按着不放就能连续加/减参数。
                // 上一次的事件还没被取走时不重复置位，天然形成"背压"，不会堆积。
                if (k->event == KEY_ACT_NONE)
                {
                    k->repeat_cnt++;
                    if (k->repeat_cnt >= KEY_REPEAT_CNT)
                    {
                        k->repeat_cnt = 0;
                        k->event      = KEY_ACT_LONG;
                    }
                }
            }
        }
        else                                    // 松开：计数清零，本次按键结束
        {
            k->press_cnt  = 0;
            k->repeat_cnt = 0;
            k->event      = KEY_ACT_NONE;
        }
    }
}

// 取一次按键事件（取出后自动清零，保证一次按键只被消费一次）
KEY_EVENT_ENUM NEUQ_Key_EventGet(void)
{
    KEY_EVENT_ENUM key_event = KEY_EVENT_NONE;

    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
        if (key_table[i].event != KEY_ACT_NONE)
        {
            // 映射逻辑：短按 = i + 1，长按 = i + 5（利用枚举顺序，一行搞定四个按键）
            if (key_table[i].event == KEY_ACT_LONG)
            {
                key_event = (KEY_EVENT_ENUM)(i + 5);
            }
            else
            {
                key_event = (KEY_EVENT_ENUM)(i + 1);
            }
            key_table[i].event = KEY_ACT_NONE;  // 关键！读取后必须清零，防止重复响应
            break;                              // 同一时刻只取一个按键
        }
    }
    return key_event;
}
