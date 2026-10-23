#include "NEUQ_lib_include.h"

typedef struct{
    GPIO_Ports_ENUM pin; //按键对应的引脚通道
    uint16_t press_cnt; //对按键的检测次数
    KEY_ACT_ENUM event; //按键自己的动作
}KEY_Typedef;

static KEY_Typedef key_table[KEY_NUM] = {
    {KEY_UP,0,KEY_ACT_NONE},
    {KEY_DOWN,0,KEY_ACT_NONE},
    {KEY_FORWARD,0,KEY_ACT_NONE},
    {KEY_BACK,0,KEY_ACT_NONE},
}; //建立并对按键表格进行初始化

void NEUQ_Key_Init(void){
    for(uint8_t i=0; i<KEY_NUM; i++){
        NEUQ_GPIO_init(key_table[i].pin,
        GPIO_Mode_Input_PullUp,GPIO_High);
    }
} //按键初始化函数

void NEUQ_Key_Scan(void){
    for(uint8_t i=0; i<KEY_NUM; i++){
        KEY_Typedef *k=&key_table[i];
        
        if(NEUQ_GPIO_read(k->pin) == GPIO_Low){
           k->press_cnt++;
           if(k->press_cnt >= KEY_LONG_CNT){
            k->event = KEY_ACT_LONG;
           }
        }else{
            if((k->press_cnt >= KEY_SHORT_CNT)&&(k->press_cnt < KEY_LONG_CNT)){
                k->event = KEY_ACT_SHORT;
            }
            k->press_cnt = 0;
        } 
    }
} //按键扫描函数

KEY_EVENT_ENUM NEUQ_Key_EventGet(void){
    KEY_EVENT_ENUM KeyEvent = KEY_EVENT_NONE;
    for(uint8_t i=0; i<KEY_NUM; i++){
        if(key_table[i].event != KEY_ACT_NONE){
            KeyEvent = (key_table[i].event == KEY_ACT_LONG?
            ((KEY_EVENT_ENUM)(i+5)):((KEY_EVENT_ENUM)(i+1)));
            key_table[i].event = KEY_ACT_NONE;
            break;
        }
    }
    return KeyEvent;
}