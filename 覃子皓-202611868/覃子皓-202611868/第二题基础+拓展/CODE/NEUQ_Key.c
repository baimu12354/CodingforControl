#include "NEUQ_lib_include.h"

typedef struct {
    GPIO_Ports_ENUM pin;
    uint8_t last_level;
    uint16_t press_cnt;
    uint8_t  debounce_cnt;
    KEY_ACT_ENUM event;
}KEY_Typedef;   //定义结构体（按键引脚，上次电平，按下持续次数，防抖，事件）

static KEY_Typedef key_table[KEY_NUM] = {
    {KEY_UP, 0,0,0, KEY_ACT_NONE},
    {KEY_DOWN, 0,0,0, KEY_ACT_NONE},
    {KEY_FORWARD, 0,0,0, KEY_ACT_NONE},
    {KEY_BACK, 0,0,0, KEY_ACT_NONE},
};  //初始化


void NEUQ_Key_Init(void){
    for(uint8_t i=0; i<KEY_NUM; i++){
        NEUQ_GPIO_init(key_table[i].pin,
        GPIO_Mode_Input_PullUp,GPIO_High);
    }
}   //按键初始化上拉，高电平
void NEUQ_Key_Scan(void){
    for (uint8_t i=0; i<KEY_NUM; i++){
        KEY_Typedef *k = &key_table[i];
        uint8_t level = (NEUQ_GPIO_read(k->pin) == GPIO_Low) ? 1 : 0;

        if (level != k->last_level){
            k->debounce_cnt++;   //当电平变化时计数增加
            if(k->debounce_cnt >= 10){
                k->debounce_cnt = 0;
                k->last_level = level;  //消抖
                if (level == 1){   //按下计数清零
                    k->press_cnt = 0;
                }else{  //松开时
                    if(k->press_cnt >= KEY_SHORT_CNT && k->press_cnt <KEY_LONG_CNT){  //短按区间
                        k->event = KEY_ACT_SHORT;  //计为短按
                    }
                    k->press_cnt = 0;  //清零
                }
            }
        }else{
            k->debounce_cnt = 0;
            if(k->last_level == 1){
                if(k->press_cnt <0xFFFF){  //防止溢出
                    k->press_cnt++;
                }
                if(k->press_cnt == KEY_LONG_CNT){
                    k->event = KEY_ACT_LONG;  //达到长按计为长按
                }
            }
        }
    }
}
KEY_EVENT_ENUM NEUQ_Key_EventGet(void){
    KEY_EVENT_ENUM KeyEvent = KEY_EVENT_NONE;
    for (uint8_t i=0; i<KEY_NUM; i++){
        if (key_table[i].event != KEY_ACT_NONE){  //判断长短按
            KeyEvent = (key_table[i].event == KEY_ACT_LONG)?
                ((KEY_EVENT_ENUM)(i+5)):((KEY_EVENT_ENUM)(i+1));
            key_table[i].event = KEY_ACT_NONE;
            break;
        }
        
    }
    return KeyEvent;
}
