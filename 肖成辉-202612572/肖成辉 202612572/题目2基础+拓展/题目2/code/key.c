#include "key.h"
typedef struct{
    GPIO_Ports_ENUM pin;
    uint16_t presscnt;
    KEY_ACT_ENUM event;


}KEY_1;
static KEY_1 key_table[4]={
    {KEY_UP,0,KEY_ACT_NONE},
    {KEY_DOWN,0,KEY_ACT_NONE},
    {KEY_FORWARD,0,KEY_ACT_NONE},
    {KEY_BACK,0,KEY_ACT_NONE},
};
void KEY_Init(void){
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
       NEUQ_GPIO_init(key_table[i].pin,GPIO_Mode_Input_PullUp,GPIO_High);
    }
    
    NEUQ_PIT_init_ms(PIT_CH1,10);
}
void KEY_Scan(void){
    for(uint8_t i=0;i<KEY_COUNT;i++){
        KEY_1 *k = &key_table[i];
        if(NEUQ_GPIO_read(k->pin) == GPIO_Low){
            k->presscnt++;
           
        }else{if(k->presscnt>=2){k->event = KEY_PRESS;

        }
        k->presscnt = 0;
      }
    }
}
KEY_EVENT_ENUM KEYGet(void)
{ KEY_EVENT_ENUM KEYevent = KEY_NONE;
for (uint8_t i = 0; i <KEY_COUNT; i++)
{if (key_table[i].event != KEY_ACT_NONE)
{
    KEYevent = (KEY_EVENT_ENUM)(i+1);
    key_table[i].event = KEY_ACT_NONE;
    break;
}

   
}
return KEYevent;
}