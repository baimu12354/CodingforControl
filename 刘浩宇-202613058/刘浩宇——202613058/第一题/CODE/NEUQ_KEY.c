#include "NEUQ_lib_include.h"

typedef struct {//按键内容
    GPIO_Ports_ENUM pin;      //引脚
    uint16_t        press_cnt;//按压时间次数
    KEY_ACT_ENUM    event;    //按钮事件
    uint8_t         key_state;//按钮状态
}KEY_TYPEDEF;

static KEY_TYPEDEF key_table[KEY_NUM]={
    {KEY_UP     ,0,KEY_ACT_NONE,0},
    {KEY_DOWN   ,0,KEY_ACT_NONE,0},
    {KEY_FORWARD,0,KEY_ACT_NONE,0},
    {KEY_BACK   ,0,KEY_ACT_NONE,0},
};

void NEUQ_KEY_INIT(void){
    for (uint8_t i = 0; i < KEY_NUM; i++){
        NEUQ_GPIO_init(
            key_table[i].pin,
            GPIO_Mode_Input_PullUp,
            GPIO_High
        );
    }
}

void NEUQ_KEY_scan(void){
    for (uint8_t i = 0; i < KEY_NUM; i++) {
        KEY_TYPEDEF *k = &key_table[i];
        if (NEUQ_GPIO_read(k->pin) == GPIO_Low) { 
            if (k->key_state == 0) {
                k->key_state = 1;
                k->press_cnt = 0;
                k->event = KEY_ACT_NONE;
            }
            k->press_cnt++;
            if ((k->press_cnt >= KEY_LONG_CNT) && (k->event != KEY_ACT_LONG)) {
                k->event = KEY_ACT_LONG;
            }
        } else { 
            if ((k->press_cnt >= KEY_SHORT_CNT) && (k->press_cnt < KEY_LONG_CNT)) {
                k->event = KEY_ACT_SHORT;
            }
            k->key_state = 0; //进入长按计时
            k->press_cnt = 0;
        }
    }
}

KEY_EVENT_ENUM NEUQ_KEY_EVENTGET(void){
    KEY_EVENT_ENUM keyevent = KEY_ACT_NONE;     //用keyevent捕捉
    for (uint8_t i = 0; i < KEY_NUM; i++){
         if(key_table[i].event != KEY_ACT_NONE){
            keyevent = (key_table[i].event == KEY_ACT_LONG)?
                ((KEY_EVENT_ENUM)(i+5)):((KEY_EVENT_ENUM)(i+1));
            key_table[i].event =KEY_ACT_NONE;
            break;
        }
    }
    return keyevent;
}