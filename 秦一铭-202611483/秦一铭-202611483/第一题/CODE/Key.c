#include "NEUQ_lib_include.h"


typedef struct{
    GPIO_Ports_ENUM pin;
    uint16_t press_cnt;
    KEY_ACT_ENUM event;
}KEY_Typedef;

static KEY_Typedef key_table[NUM] = {
    {MODE1,0,KEY_ACT_NONE},
    {MODE2,0,KEY_ACT_NONE},
    {MODE3,0,KEY_ACT_NONE},
    {MODE4,0,KEY_ACT_NONE},
};

//初始化
void NEUQ_Key_Init(void){
    for (uint8_t i = 0; i < NUM; i++)
    {
        NEUQ_GPIO_init(key_table[i].pin,
            GPIO_Mode_Input_PullUp,GPIO_High);
    }
    
}

//按键消抖
void NEUQ_Key_Scan(void){
    for (uint8_t i = 0; i < NUM; i++)
    {
        KEY_Typedef*k = &key_table[i];
        if (NEUQ_GPIO_read(k->pin) == GPIO_Low)
        {k->press_cnt++;
            if (k->press_cnt >= SHORT_CNT)
            {
                k->event = KEY_ACT_SHORT;
            }
            
        }else k->press_cnt = 0;
        
    }
    
}

//读取并清除按键事件
KEY_EVENT_ENUM NEUQ_Key_EventGet(void){
    KEY_EVENT_ENUM KeyEvent = EVENT_NONE;
    for(uint8_t i=0;i<NUM;i++){
        if(key_table[i].event == KEY_ACT_SHORT){
            KeyEvent = (KEY_EVENT_ENUM)(i+1);
            key_table[i].event = KEY_ACT_NONE;
            break;
        }
    }
    return KeyEvent;
}