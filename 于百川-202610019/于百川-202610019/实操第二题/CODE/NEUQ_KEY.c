#include"NEUQ_lib_include.h"

typedef struct{
    /* data */
    GPIO_Ports_ENUM pin;
    uint16_t press_cnt;
    uint8_t pressed;
}KEY_Typedef;

static KEY_Typedef key_table[KEY_NUM] = {
    {KEY_UP_PIN,       0, 0},
    {KEY_DOWN_PIN,     0, 0},
    {KEY_FORWARD_PIN,  0, 0},
    {KEY_BACK_PIN,     0, 0},
};

static KEY_EVENT_ENUM key_event = KEY_NONE;   // 单事件缓冲 

void NEUQ_KEY_Init(void){
    NEUQ_GPIO_init(KEY_UP_PIN,      GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_DOWN_PIN,    GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_FORWARD_PIN, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_BACK_PIN,    GPIO_Mode_Input_PullUp, GPIO_High);
}

//AI
void NEUQ_KEY_Scan(void){
    for (uint8_t i=0; i<KEY_NUM; i++){
        KEY_Typedef *k = &key_table[i];
        uint8_t raw = (NEUQ_GPIO_read(k->pin) == KEY_PRESS_LEVEL) ? 1 : 0;

        if (raw == k->pressed){
            k->press_cnt = 0;                    
        } else if (++k->press_cnt >= KEY_DEBOUNCE_MS){  
            k->press_cnt = 0;
            k->pressed   = raw;
            if (raw == 1 && key_event == KEY_NONE){    
                key_event = (KEY_EVENT_ENUM)(KEY_UP + i);
            }
        }  
    }
}

KEY_EVENT_ENUM NEUQ_KEY_Get(void){
    KEY_EVENT_ENUM evt = key_event;
    key_event = KEY_NONE;
    return evt;
}