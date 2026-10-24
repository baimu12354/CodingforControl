#include "NEUQ_lib_include.h"

const uint8_t g_pattern[4][10]={
    {1,1,1,1,1,0,0,0,0,0},
    {1,0,0,0,0,0,0,0,0,0},
    {1,0,1,0,0,0,0,0,0,0},
    {1,0,1,1,1,0,0,0,0,0}
};

volatile uint8_t g_cur_scheme=0;
volatile uint8_t g_pending_scheme=0;
volatile uint8_t g_step=0;
volatile uint8_t  g_step_tick      = 0;
volatile uint8_t  g_cycle_done     = 0; 
volatile uint8_t  g_key_pressed=0;
void LED_init(void)
{
    NEUQ_GPIO_init(LED_Pin,GPIO_Mode_Output_PushPull,GPIO_Low);
}

void LED_Tick(void){
    g_step_tick++;
    if(g_step_tick>=200){
        g_step+=1;
        g_step_tick=0;
    }
    if(g_step>=10){
        g_step=0;
        g_step_tick=0;
        g_cycle_done=1;
        if(g_key_pressed==0)
        g_cur_scheme = (g_cur_scheme + 1) % 4; 
    }
    NEUQ_GPIO_write(LED_Pin,g_pattern[g_cur_scheme][g_step]?GPIO_High:GPIO_Low);
    // if(++g_step>=10){
    //     g_step=0;
    //     g_pending_scheme = (g_pending_scheme + 1) % 4;
    //     g_cur_scheme=g_pending_scheme;
    // }
}

void scheme_update(void){
    if(g_cycle_done==1)
    {
        g_cycle_done=0;
        if(g_cur_scheme!=g_pending_scheme&&g_key_pressed==1)
        {
            g_cur_scheme=g_pending_scheme;
            g_step=0;
            g_step_tick=0;
        }
        g_key_pressed = 0;
        g_pending_scheme = (g_cur_scheme + 1) % 4; 
    }
}