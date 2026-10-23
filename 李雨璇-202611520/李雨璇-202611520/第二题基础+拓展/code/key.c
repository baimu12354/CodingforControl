#include "NEUQ_lib_include.h"

typedef struct{
    GPIO_Ports_ENUM  pin;         //按键对应引脚
    uint16_t         press_cnt;   //按了几个计数周期
    KEY_STATE        state;       //按键状态
}KEY_ID;

static KEY_ID key_table[KEY_NUM]={
    {KEY_UP,0,KEY_STATE_NONE},
    {KEY_DOWN,0,KEY_STATE_NONE},
    {KEY_FORWARD,0,KEY_STATE_NONE},
    {KEY_BACK,0,KEY_STATE_NONE},
};

void Key_Init(void){
    for(uint8_t i=0; i<KEY_NUM; i++){
        NEUQ_GPIO_init(key_table[i].pin,GPIO_Mode_Input_PullUp,GPIO_High);
    }
}

void Key_Scan(void){
    for(uint8_t i=0; i<KEY_NUM; i++){
        KEY_ID *k  = &key_table[i];

        if (NEUQ_GPIO_read(k->pin) == GPIO_Low){
            k->press_cnt++;
            if (k->press_cnt >= KEY_L_PRESS){
                k->state = KEY_STATE_L;
            }
        }       else{
            if((k->press_cnt >= KEY_S_PRESS)&&(k->press_cnt < KEY_L_PRESS)){
                k->state = KEY_STATE_S;
                }
             k->press_cnt = 0;
        }
    }
}

KEY_EVENT Key_EventGet(void){
    KEY_EVENT KeyEvent = KEY_EVENT_NONE;
    for (uint8_t i=0; i<KEY_NUM; i++){
        if(key_table[i].state != KEY_STATE_NONE){
            KeyEvent = (key_table[i].state == KEY_STATE_L)?
            ((KEY_EVENT)(i+5)):((KEY_EVENT)(i+1));
            key_table[i].state = KEY_STATE_NONE;
            break;
        }
    }
return KeyEvent;
}