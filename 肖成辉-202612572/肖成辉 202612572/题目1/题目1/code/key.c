#include "key.h"

static int presscnt = 0;
static KEY_ACT_ENUM keyflag = KEY_ACT_NONE;

void KEY_Init(void)
{
    NEUQ_GPIO_init(KEY, GPIO_Mode_Input_PullUp, GPIO_Low);
    NEUQ_PIT_init_ms(PIT_CH1, 10);
}

void KEY_Scan(void)
{
    if (NEUQ_GPIO_read(KEY) == GPIO_Low)
    {
        presscnt++;
        if (presscnt >= 50)
        {
            keyflag = KEY_PRESS_SHORT;
        }
    }
    else
    {
        if ((presscnt >= 2) && (presscnt <= 50))
        {
            keyflag = KEY_PRESS_LONG;
        }
        presscnt = 0;
    }
}

KEY_EVENT_ENUM KEY_Get(void)
{
    KEY_EVENT_ENUM event = KEY_NONE;

    if (keyflag != KEY_ACT_NONE)
    {
        event = KEY_PRESS;
        keyflag = KEY_ACT_NONE;
    }

    return event;
}