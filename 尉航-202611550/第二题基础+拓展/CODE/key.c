#include "key.h"


typedef struct{
    GPIO_Ports_ENUM pin;
    uint16_t press_cnt;
    KEY_ACT_ENUM event;
}KEY_Typedef;

static KEY_Typedef Key_table[KEY_NUM] = {
    {KEY_UP,0,KEY_ACT_NONE},
    {KEY_DOWN,0,KEY_ACT_NONE},
    {KEY_FORWARD,0,KEY_ACT_NONE},
    {KEY_BACK,0,KEY_ACT_NONE},
};
//初始化代码
void Key_Init(void){
    for(uint8_t i = 0; i<KEY_NUM;i++){
        NEUQ_GPIO_init(Key_table[i].pin,GPIO_Mode_Input_PullUp,GPIO_High); //给每个按键配置为上拉
    }
};


void Key_Scan(void){
    for(uint8_t i = 0;i<KEY_NUM;i++){
        KEY_Typedef *k = &Key_table[i]; //取出每个按键状态的地址
        
        //检测按键状态
        if(NEUQ_GPIO_read(k->pin) == GPIO_Low){
            k->press_cnt++;
            if(k->press_cnt >= KEY_LONG_CNT){
                k->event = KEY_ACT_LONG;
            }
        }else{
            if((k->press_cnt >= KEY_SHORT_CNT) && (k->press_cnt <KEY_LONG_CNT)){
                k->event = KEY_ACT_SHORT;
            }
            k->press_cnt = 0; //检测完后将按键次数清空
        }
    }
};



KEY_EVENT_ENUM Key_EventGet(void){
    KEY_EVENT_ENUM keyEvent = KEY_EVENT_NONE;//定义返回值 

    //遍历循环
    for(uint8_t i = 0; i<KEY_NUM;i++){
        //如果检测到按键有动作，给返回值赋值
        if(Key_table[i].event != KEY_ACT_NONE){
            keyEvent = (Key_table[i].event == KEY_ACT_LONG)?
                ((KEY_EVENT_ENUM)(i+5)) : ((KEY_EVENT_ENUM)(i+1));
            Key_table[i].event = KEY_ACT_NONE;//清除按键状态
            break;
        }
    }
    return keyEvent;
};
