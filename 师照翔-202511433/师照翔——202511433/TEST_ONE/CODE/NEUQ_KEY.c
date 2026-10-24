#include "NEUQ_lib_include.h"

static KEY_Typedef Key_table[KEY_NUM]={
    {KEY_UP,0,KEY_ACT_NONE},
    {KEY_DOWN,0,KEY_ACT_NONE},
    {KEY_FORWORD,0,KEY_ACT_NONE},
    {KEY_BACK,0,KEY_ACT_NONE},
};

void NEUQ_Key_Init(void){
    for(uint8_t i=0;i<KEY_NUM;i++){
        NEUQ_GPIO_init(Key_table[i].Pin,GPIO_Mode_Input_PullUp,GPIO_High);
    }
}
void NEUQ_Key_Scan(void){
    for(uint8_t i=0;i<KEY_NUM;i++){
        KEY_Typedef *k=&Key_table[i];

        if(NEUQ_GPIO_read(k->Pin)==GPIO_Low){
            k->press_cnt++;
            if(k->press_cnt>KEY_LONG_CNT){
                k->event=KEY_ACT_LONG;
            }
        }else{
            if(k->press_cnt>KEY_SHORT_CNT&&k->press_cnt<=KEY_LONG_CNT){
                k->event=KEY_ACT_SHORT;
               }
            k->press_cnt=0;
        }
    }
}
KEY_EVENT_ENUM NEUQ_Key_Event(void)
{
    KEY_EVENT_ENUM respond=KEY_EVENT_NONE;
    for(uint8_t i=0;i<KEY_NUM;i++){
        if(Key_table[i].event != KEY_ACT_NONE){
           respond=((Key_table[i].press_cnt>KEY_LONG_CNT)?(KEY_EVENT_ENUM)(i+5):(KEY_EVENT_ENUM)(i+1));
           Key_table[i].event = KEY_ACT_NONE;
           break;
        }
    }
    return respond;
}