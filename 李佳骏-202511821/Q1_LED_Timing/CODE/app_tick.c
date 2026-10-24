/* app_tick.c - 10 ms 系统节拍 */

#include "app_tick.h"

volatile uint32_t g_tick_10ms = 0;

void AppTick_Init(void)
{
    g_tick_10ms = 0;
}

void AppTick_OnPitInterrupt(void)
{
    g_tick_10ms++;
}

uint32_t AppTick_Get(void)
{
    return g_tick_10ms;
}
