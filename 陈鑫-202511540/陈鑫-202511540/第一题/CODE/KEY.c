#include "NEUQ_lib_include.h"

static uint8_t g_key_last=1;
static uint16_t g_key_debounce=0;
static volatile uint8_t key_event = 0;
static uint8_t key_state=1;

void KEY_init(void)
{
    NEUQ_GPIO_init(KEY_Pin,GPIO_Mode_Input_PullUp,GPIO_High);
}

void Key_Scan(void){
    uint8_t now=(NEUQ_GPIO_read(KEY_Pin)==KEY_ACTIVE)?0:1;
    if(now!=g_key_last)
    {
        g_key_debounce=0;
    }else{
        if(g_key_debounce<20)
        g_key_debounce++;
    }
    if(g_key_debounce>=20){
        if(key_state!=now){
            key_state=now;
            if(key_state==0){
                key_event=1;
            }
        }
    }
    g_key_last=now;

    // uint32_t now=NEUQ_Get_Timer_Value_ms();
    // if(raw!=g_key_last){
    //     g_key_last=raw;
    //     g_key_chg_tick=now;
    //     return;
    // }
    // if((now-g_key_chg_tick)>=20){
    //     if(raw==0&&g_key_level==1)
    //     {
    //         g_key_level=0;
    //         g_pending_scheme=(g_pending_scheme+1)%4;
    //     }else if(raw==1&&g_key_level==0)
    //     {
    //         g_key_level=1;
    //     }
    // }
}

void KEY_Process(void){
    if(key_event){
        key_event=0;
        g_pending_scheme=(g_pending_scheme+1)%4;
        g_key_pressed=1;
    }
}