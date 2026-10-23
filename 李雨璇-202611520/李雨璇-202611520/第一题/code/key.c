#include "NEUQ_lib_include.h"

typedef struct{
    uint16_t press_cnt;   //连续按下的毫秒计数
    uint8_t  event;       //0:当前未按下  1:当前已按下
}KEY_ID;

static const GPIO_Ports_ENUM key_pins[KEY_NUM] = { KEY_P_1, KEY_P_2, KEY_P_3, KEY_P_4 };
static KEY_ID key_table[KEY_NUM];

void Key_Init(void){
    for (uint8_t i = 0; i < KEY_NUM; i++){
        NEUQ_GPIO_init(key_pins[i], GPIO_Mode_Input_PullUp, GPIO_High);
    }
}

void Key_Scan(void){
    for (uint8_t i = 0; i < KEY_NUM; i++){
        KEY_ID *k = &key_table[i];

        if (NEUQ_GPIO_read(key_pins[i]) == GPIO_Low){
            k->press_cnt++;

            if (k->press_cnt >= KEY_ON_MS){
                k->event   = 1;
            }
        }
        else{
            k->press_cnt = 0;
        }
    }
}

KEY_EVENT Key_EventGet(void){
    for (uint8_t i = 0; i < KEY_NUM; i++){
        if (key_table[i].event == 1){
            key_table[i].event = 0;
            return (KEY_EVENT)(i + 1);
        }
    }
    return KEY_EVENT_NONE;
}