#include "NEUQ_lib_include.h"

typedef struct
{
    GPIO_Ports_ENUM pin;
    uint16_t press_cut;
    KEY_ACT_ENUM event;
    uint8_t long_sent;               // 本次按下已触发过长按，松开时不再补发短按
}KEY_Typedef;

static KEY_Typedef Key_table[KEY_NUM]={
    {KEY_UP,0,KEY_ACT_NONE,0},
    {KEY_DOWN,0,KEY_ACT_NONE,0},
    {KEY_FORWARD,0,KEY_ACT_NONE,0},
    {KEY_BACK,0,KEY_ACT_NONE,0}
};
void NEUQ_KEY_Init(void){
    for(uint8_t i=0;i<KEY_NUM;i++){
        NEUQ_GPIO_init(Key_table[i].pin,GPIO_Mode_Input_PullUp,GPIO_High);
    }
}
void NEUQ_KEY_Scan(void){
    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
        KEY_Typedef *k=&Key_table[i];
        if (NEUQ_GPIO_read(k->pin)==GPIO_Low)          // 按下（低电平）
        {
                k->press_cut++;
            if (k->press_cut>=KEY_LONG_GNT)            // 长按
            {
                k->event=KEY_ACT_LONG;
                k->press_cut=0;                        // 触发后清零，允许长按连发
                k->long_sent=1;                        // 标记长按，松开不再补发短按
            }
        }
        else                                            // 松开
        {
            if ((!k->long_sent) && (k->press_cut>=KEY_SHORT_CNT) && (k->press_cut<KEY_LONG_GNT))
            {
                k->event=KEY_ACT_SHORT;                // 仅真正的短按才发短按
            }
                k->press_cut=0;
                k->long_sent=0;
        }
    }
}
KEY_EVENT_ENUM NEUQ_KEY_EventGet(void){
    KEY_EVENT_ENUM KEY_Event=KEY_EVENT_NONE;
    for(uint8_t i=0;i<KEY_NUM;i++){
        if(Key_table[i].event!=KEY_ACT_NONE){
            KEY_Event=(Key_table[i].event==KEY_ACT_LONG)?
            ((KEY_EVENT_ENUM)(i+5)):((KEY_EVENT_ENUM)(i+1));
            Key_table[i].event=KEY_ACT_NONE;
            break;
        }
    }
    return KEY_Event;
}
