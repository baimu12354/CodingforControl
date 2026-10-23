#include "NEUQ_lib_include.h"


void NEUQ_Key_init(void){
    NEUQ_GPIO_init(NEUQ_KEY_PIN, GPIO_Mode_Input_PullUp, GPIO_High);  //上拉，默认高电平
}

uint8_t NEUQ_Key_scan(void){
    static uint8_t last_key     = 0;
    static uint8_t debounce_cnt = 0;
    uint8_t key_now = NEUQ_KEY_READ();

    if (key_now != last_key){  //按键消抖
        debounce_cnt++;
        if (debounce_cnt >= 10){
            debounce_cnt = 0;
            last_key = key_now;
            if (key_now == 1){
                return 1;
            }
        }

    }else{
        debounce_cnt = 0;
    }
    return 0;
}

