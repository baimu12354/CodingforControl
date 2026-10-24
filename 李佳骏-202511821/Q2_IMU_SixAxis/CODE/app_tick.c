/* app_tick.c - 1 ms 系统节拍 */

#include "app_tick.h"
#include "board_config.h"

volatile uint32_t g_tick_ms = 0;

static volatile uint8_t s_flags = 0;

void AppTick_Init(void)
{
    g_tick_ms = 0;
    s_flags   = 0;
}

/* 1 ms 中断里调用：累加毫秒，按分频置采样/上传/刷屏标志 */
uint8_t AppTick_OnPitIsr(void)
{
    uint32_t now;

    g_tick_ms++;
    now = g_tick_ms;

    if ((now % T2_SAMPLE_TICKS) == 0U)
    {
        s_flags |= APP_FLAG_SAMPLE;
    }
    if ((now % T2_UPLOAD_TICKS) == 0U)
    {
        s_flags |= APP_FLAG_UPLOAD;
    }
    if ((now % T2_UI_TICKS) == 0U)
    {
        s_flags |= APP_FLAG_UI;
    }
    if ((now % T2_KEY_SCAN_TICKS) == 0U)
    {
        return 1U;
    }
    return 0U;
}

/* 取走标志，取到返回 1 */
uint8_t AppTick_TakeFlag(uint8_t flag)
{
    if ((s_flags & flag) != 0U)
    {
        s_flags &= (uint8_t)(~flag);
        return 1U;
    }
    return 0U;
}

uint32_t AppTick_GetMs(void)
{
    return g_tick_ms;
}
