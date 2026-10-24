#include "NEUQ_lib_include.h"

typedef struct{
    uint16_t press_cnt;
    uint8_t  press_state;
    uint8_t  press_event;
}KEY_Typedef;

static KEY_Typedef key;

void NEUQ_Key_Init(void){
    NEUQ_GPIO_init(PA0, GPIO_Mode_Input_PullUp, GPIO_High);
}

void NEUQ_Key_Scan(void){
    uint8_t key_pin = NEUQ_GPIO_read(PA0);
    if(key_pin == GPIO_Low){
        key.press_cnt++;
        if(key.press_cnt >=5){
            key.press_state = 1;
        }
    }
    else{
        if(key.press_state == 1 ){
            key.press_event = 1;
        }
        key.press_state = 0;
        key.press_cnt = 0;
    }
}

uint8_t NEUQ_Key_EventGet(void){
    uint8_t temp = key.press_event;
    key.press_event = 0;
    return temp;
}
