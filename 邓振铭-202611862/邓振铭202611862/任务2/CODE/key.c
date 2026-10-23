#include"NEUQ_lib_include.h"
#include"key.h"
typedef struct
{
    /* data */
    GPIO_Ports_ENUM pin;
    uint16_t press_cnt;   // 消抖计数
    uint8_t  latched;     // 锁存：一次按下只上报一次
    KEY_ACT_ENUM event;
}KEY_TYDE;

 KEY_TYDE key_table[]={
    {UP,0,0,KEY_ACT_NONE},
    {DOWN,0,0,KEY_ACT_NONE},
    {FORWARD,0,0,KEY_ACT_NONE},
    {BACK,0,0,KEY_ACT_NONE},
    {RIGHT,0,0,KEY_ACT_NONE}
};
void key_init(void){
for(uint16_t i=0;i<=4;i++){
    NEUQ_GPIO_init(key_table[i].pin,GPIO_Mode_Input_PullUp,GPIO_High);
}
}
void key_scan(void){
    for(uint8_t i=0;i<=4;i++){
        KEY_TYDE *k=&key_table[i];
        if(NEUQ_GPIO_read(k->pin)==GPIO_Low){      // 按下
            if(k->press_cnt<2) k->press_cnt++;
            if(k->press_cnt>=2 && k->latched==0){  // 消抖通过且本次按下尚未上报
                k->event=KEY_ACT_HAVE;
                k->latched=1;
            }
        }else{                                      // 松开
            k->press_cnt=0;
            k->latched=0;
        }
    }
}
uint16_t key_eventget(void){
for(uint8_t i=0;i<=4;i++){
    if(key_table[i].event==KEY_ACT_HAVE){
        key_table[i].event=KEY_ACT_NONE;
        return i;
    }
}
return 0xFF;
}
