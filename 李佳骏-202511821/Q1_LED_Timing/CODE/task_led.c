/* task_led.c - 第一题任务入口 */

#include "task_led.h"
#include "board_config.h"
#include "app_tick.h"
#include "bsp_key.h"
#include "led_pattern.h"
#include "ui_led.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

/* 长按每 100 ms 连续切换一次 */
#define T1_LONG_REPEAT_TICKS    (10U)

static uint32_t s_last_repeat_tick = 0;

#if (T1_SHOW_KEY_LEVELS != 0)
static uint32_t s_last_key_debug_tick = 0;
#endif

void Task_LED_Init(void)
{
    NEUQ_Debug_init();              /* 调试串口，LCD 驱动内部会用到 printf */
    NEUQ_Delay_init();
    NEUQ_LCD200_init();

    Key_Init();
    AppTick_Init();
    LedPattern_Init();

    Ui_Led_DrawAll();

    NEUQ_PIT_init_ms(PIT_CH1, T1_TICK_MS);      /* 启动 10 ms 节拍 */
}

/* 按键只改 pending，长按按 100 ms 限流 */
static void Task_LED_HandleKey(void)
{
    uint32_t    now = AppTick_Get();
    key_event_t ev;

    /* UP：上一套 */
    ev = Key_GetEvent(KEY_ID_UP);
    if (ev == KEY_EVENT_SHORT)
    {
        LedPattern_SelectPrev();
        s_last_repeat_tick = now;
    }
    else if ((ev == KEY_EVENT_LONG) &&
             ((uint32_t)(now - s_last_repeat_tick) >= T1_LONG_REPEAT_TICKS))
    {
        LedPattern_SelectPrev();
        s_last_repeat_tick = now;
    }

    /* DOWN：下一套 */
    ev = Key_GetEvent(KEY_ID_DOWN);
    if (ev == KEY_EVENT_SHORT)
    {
        LedPattern_SelectNext();
        s_last_repeat_tick = now;
    }
    else if ((ev == KEY_EVENT_LONG) &&
             ((uint32_t)(now - s_last_repeat_tick) >= T1_LONG_REPEAT_TICKS))
    {
        LedPattern_SelectNext();
        s_last_repeat_tick = now;
    }
}

void Task_LED_Loop(void)
{
    Task_LED_HandleKey();

    /* 待执行方案被改动、或周期边界上真的切换了方案时，重画一次 */
    if (LedPattern_TakeUiDirty())
    {
        Ui_Led_DrawAll();
    }

#if (T1_SHOW_KEY_LEVELS != 0)
    /* 调试：每 100 ms 刷新一次按键电平 */
    {
        uint32_t now = AppTick_Get();
        if ((uint32_t)(now - s_last_key_debug_tick) >= 10U)
        {
            s_last_key_debug_tick = now;
            Ui_Led_RefreshKeyLevels();
        }
    }
#endif
}
