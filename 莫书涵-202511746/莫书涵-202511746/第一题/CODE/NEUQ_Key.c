#include "NEUQ_lib_include.h"

typedef struct{
    GPIO_Ports_ENUM pin;
    uint16_t press_cnt;
    KEY_ACT_ENUM event;
}KEY_Typedef;

static KEY_Typedef key_table[KEY_NUM] = {
    {KEY_UP,0,KEY_ACT_NONE},
    {KEY_DOWN,0,KEY_ACT_NONE},
    {KEY_FORWORD,0,KEY_ACT_NONE},
    {KEY_BACK,0,KEY_ACT_NONE},
};

void NEUQ_Key_Init(void){
    for(uint8_t i = 0 ; i < KEY_NUM ; i++){
        NEUQ_GPIO_init(key_table[i].pin,GPIO_Mode_Input_PullUp,GPIO_High);
    }
}

void NEUQ_Key_Scan(void){
    for(uint8_t i = 0 ; i < KEY_NUM ; i++){
        KEY_Typedef *k = &key_table[i];

        if(NEUQ_GPIO_read(k->pin) == GPIO_Low){        // 按键按下
            if(k->press_cnt < KEY_LONG_CNT){           // 防止计数溢出
                k->press_cnt++;
            }
            if(k->press_cnt == KEY_LONG_CNT){          // 达到长按阈值，触发一次长按
                k->event = KEY_ACT_LONG;
            }
        }
        else{                                          // 按键松开
            if((k->press_cnt >= KEY_SHORT_CNT) && (k->press_cnt < KEY_LONG_CNT)){
                k->event = KEY_ACT_SHORT;              // 松开时未达长按阈值，判定为短按
            }
            k->press_cnt = 0;                          // 松开后才清零
        }
    }
}

KEY_EVENT_ENUM NEUQ_Key_EventGet(void){
    KEY_EVENT_ENUM KeyEvent = KEY_EVENT_NONE;
    for(uint8_t i = 0 ; i < KEY_NUM ; i++){
        if(key_table[i].event != KEY_ACT_NONE){
            KeyEvent = (key_table[i].event == KEY_ACT_LONG)?
                ((KEY_EVENT_ENUM)(i+5)) : ((KEY_EVENT_ENUM)(i+1));
            key_table[i].event = KEY_ACT_NONE;
            break;
        }
    }
    return KeyEvent;
}
