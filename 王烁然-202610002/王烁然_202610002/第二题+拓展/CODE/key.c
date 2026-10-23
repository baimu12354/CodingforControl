#include"NEUQ_lib_include.h"

typedef struct 
{
    GPIO_Ports_ENUM pin;
    uint16_t pressCount;
    KEY_ACT_ENUM  event;
}KEY_Typedef;


static KEY_Typedef key_table[KEY_NUM]=
{
    {KEY_UP,0,KEY_ACT_NULL},
    {KEY_DOWN,0,KEY_ACT_NULL},
    {KEY_FORWARD,0,KEY_ACT_NULL},
    {KEY_BACK,0,KEY_ACT_NULL},

};


void key_init(void)
{
    for(uint8_t i=0;i<KEY_NUM;i++)
    {
        NEUQ_GPIO_init(key_table[i].pin,GPIO_Mode_Input_PullUp,1);
    }
}


void key_scan(void)
{
    for(uint8_t i=0;i<KEY_NUM;i++)
    {
        KEY_Typedef* k = &key_table[i];
        if(NEUQ_GPIO_read(k->pin)==0)
        {
            k->pressCount++;
            if(k->pressCount>=KEY_LONG_CNT)
            {
                k->event=KEY_ACT_LONG;
            }
            
        }
        else
        {
            if((k->pressCount>=KEY_SHORT_CNT)&&(k->pressCount<KEY_LONG_CNT))
            {
                k->event=KEY_ACT_SHORT;
            }
            k->pressCount=0;
        }
    }
}


KEY_EVENT_ENUM key_eventGet(void)
{
    KEY_EVENT_ENUM event=KEY_EVENT_NULL;
    for(uint8_t i=0;i<KEY_NUM;i++)
    {
        if(key_table[i].event!=KEY_ACT_NULL)
        {
            event=(key_table[i].event==KEY_ACT_SHORT)?(KEY_EVENT_ENUM)(i+1):(KEY_EVENT_ENUM)(i+5);
            key_table[i].event=KEY_ACT_NULL;

        }
    }
    return event;
}