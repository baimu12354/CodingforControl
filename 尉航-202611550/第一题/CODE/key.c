#include "key.h"

static int key_cnt = 0;
static KEY_ACT_ENUM key_act = KEY_ACT_NONE;
//初始化：按键调为高电平
void KEY_Init(void){
    NEUQ_GPIO_init(KEY,GPIO_Mode_Input_PullUp,GPIO_High);
    NEUQ_PIT_init_ms(PIT_CH1,10);   //每10ms进行一次中断回调
};

//检测按键状态
void KEY_Scan(void){
    if(NEUQ_GPIO_read(KEY) == GPIO_Low){
        key_cnt++;
        if(key_cnt >=50){
            key_act = KEY_PRESS_LONG;
        }
        
    }else{
        if((key_cnt >= 2) && (key_cnt <50)){
            key_act = KEY_PRESS_SHORT;
        }
        key_cnt = 0;//松开按键后计数器清零
    }
    
};




//获取按键状态,获取后状态清除
KEY_EVENT_ENUM KEY_Get(void){
    KEY_EVENT_ENUM key_event = KEY_EVENT_NONE;
    if(key_act != KEY_ACT_NONE){
        key_event = KEY_EVENT_PRESS;
        key_act = KEY_ACT_NONE;
    }
    return key_event;
};
