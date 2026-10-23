#include "NEUQ_lib_include.h"

uint8_t  g_cur  = 0;   /* 正在执行的方案 */
uint8_t  g_next = 0;   /* 待执行的方案（按键只改它） */
uint8_t  g_seg = 0;             /* 当前走到第几段（只有中断碰） */
uint16_t g_ms  = 0;             /* 当前段内走了多少毫秒（只有中断碰） */
extern uint8_t tag;

void LED_Init(void)
{
    NEUQ_GPIO_init(LED,GPIO_Mode_Output_PushPull,GPIO_Low);
}

void LED_Tick(void)
{
    if (++g_ms < SCHEME[g_cur].seg[g_seg].ms)
        return;                                /* 本段还没走完 */

    g_ms = 0;                                 /* 本段结束，归零 */
    
    if (++g_seg >= SCHEME[g_cur].nseg)         /* 段号+1，走到表尾？ */
    {
        
        g_seg = 0;                             /* 回到第1段 = 2s边界 */
        if (g_next != g_cur)                   /* 只在此处换方案 */
            g_cur = g_next;
            SHOW_SCREEN_WORK(&PAGE);
    }
    if (SCHEME[g_cur].seg[g_seg].lvl) NEUQ_GPIO_write(LED, GPIO_High);   /* 按新段电平点灯 */
    else                              NEUQ_GPIO_write(LED, GPIO_Low);
}
