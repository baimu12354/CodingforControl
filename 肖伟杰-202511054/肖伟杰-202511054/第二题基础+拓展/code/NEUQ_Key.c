/*********************************************************************************************************************
* 文件名称          NEUQ_Key
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              按键驱动（第一题 LED 时序控制）：计数消抖 + 短按/长按事件 + 表格驱动
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "NEUQ_Key.h"

/* ------------------------- 按键状态表 ------------------------- */
typedef struct
{
    GPIO_Ports_ENUM pin;         /* 按键引脚（上拉输入，按下为低） */
    uint8_t         press_cnt;   /* 按住累计次数（10 ms 一次） */
    uint8_t         repeat_cnt;  /* 长按重复计数（10 ms 一次） */
    uint8_t         event;       /* 事件槽：KEY_EVENT_NONE / SHORT / LONG */
} KeyItemType;

static KeyItemType key_table[KEY_ID_NUM] =
{
    { PE9,  0, 0, KEY_EVENT_NONE },    /* KEY_ID_UP      */
    { PE7,  0, 0, KEY_EVENT_NONE },    /* KEY_ID_DOWN    */
    { PE8,  0, 0, KEY_EVENT_NONE },    /* KEY_ID_FORWARD */
    { PE10, 0, 0, KEY_EVENT_NONE },    /* KEY_ID_BACK    */
    { PA0,  0, 0, KEY_EVENT_NONE },    /* KEY_ID_K0      */
};

/* 读引脚：默认读真实 GPIO；主机侧验证（tools/task1_verify）可以替换成模拟电平，
 * 这样消抖逻辑能在 PC 上直接跑，不用接板子。 */
static GPIO_Level Key_ReadPin(GPIO_Ports_ENUM pin)
{
    return NEUQ_GPIO_read(pin);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     按键初始化：全部配置为上拉输入（松开为高，按下为低）
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Key_Init(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)KEY_ID_NUM; i++)
    {
        key_table[i].press_cnt  = 0U;
        key_table[i].repeat_cnt = 0U;
        key_table[i].event      = KEY_EVENT_NONE;
        NEUQ_GPIO_init(key_table[i].pin, GPIO_Mode_Input_PullUp, GPIO_High);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     按键扫描：每 10 ms 调用一次
* 备注信息     计数消抖：连续采到低电平就累加，采到高电平就结束累加并清零。
*              短按在"松手"时确认；长按到达门限立刻响应，之后按固定节奏重复响应。
*              这里只写事件槽，不碰 LED 时序，也不做任何耗时操作（由 1 ms 节拍分频调用）。
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Key_Scan(void)
{
    uint8_t      i;
    KeyItemType *key;

    for (i = 0U; i < (uint8_t)KEY_ID_NUM; i++)
    {
        key = &key_table[i];

        if (Key_ReadPin(key->pin) == GPIO_Low)             /* ------ 采到按下 ------ */
        {
            if (key->press_cnt < KEY_CNT_MAX)
            {
                key->press_cnt++;
            }

            if (key->press_cnt == KEY_LONG_CNT)            /* 刚好到达长按门限：立刻响应一次 */
            {
                key->event      = KEY_EVENT_LONG;
                key->repeat_cnt = 0U;
            }
            else if (key->press_cnt > KEY_LONG_CNT)        /* 之后按固定节奏重复响应 */
            {
                key->repeat_cnt++;
                if (key->repeat_cnt >= KEY_LONG_REPEAT_CNT)
                {
                    key->repeat_cnt = 0U;
                    key->event      = KEY_EVENT_LONG;
                }
            }
            else
            {
                /* 未到门限：只累计，不发任何事件（还分不清是短按还是长按） */
            }
        }
        else                                              /* ------ 采到松开 ------ */
        {
            if ((key->press_cnt >= KEY_DEBOUNCE_CNT) && (key->press_cnt < KEY_LONG_CNT))
            {
                key->event = KEY_EVENT_SHORT;              /* 松手时才确认本次短按 */
            }
            /* 已按满长按门限的正常长按，松手后不再补发短按 */

            key->press_cnt  = 0U;                          /* 清零，为下一次做准备 */
            key->repeat_cnt = 0U;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     读取并清除某个按键的事件（读后清零，不是队列）
*------------------------------------------------------------------------------------------------------------------*/
KeyEventEnum NEUQ_Key_GetEvent(KeyIdEnum id)
{
    KeyEventEnum event;

    if ((uint8_t)id >= (uint8_t)KEY_ID_NUM)
    {
        return KEY_EVENT_NONE;
    }

    event = (KeyEventEnum)key_table[id].event;
    key_table[id].event = KEY_EVENT_NONE;
    return event;
}
