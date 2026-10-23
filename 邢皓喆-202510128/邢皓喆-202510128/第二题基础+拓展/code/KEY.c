#include "KEY.h"

KEY_TypeDef KEY[KEY_NUM] = {
    {KEY_UP, 0, KEY_ACT_NONE},
    {KEY_DOWN, 0, KEY_ACT_NONE},
    {KEY_FORWARD, 0, KEY_ACT_NONE},
    {KEY_BEHIND, 0, KEY_ACT_NONE}
};
void KEY_Init(void){
    uint8_t i;
  for( i = 0; i < KEY_NUM; i++){
    NEUQ_GPIO_init(KEY[i].KEY_PIN, GPIO_Mode_Input_PullUp, GPIO_High); 
}
}
void KEY_Scan(void)
{
    uint8_t i;
    static uint8_t initialized = 0;
    static uint8_t stable_level[KEY_NUM];
    if(!initialized)
    {
        for(i = 0; i < KEY_NUM; i++)
        {
            stable_level[i] = GPIO_High;
        }
            initialized = 1;
        }
    static uint8_t current_level[KEY_NUM];
    static uint8_t wave_cnt[KEY_NUM] = {0};
    for(i = 0; i < KEY_NUM; i++){
        KEY_TypeDef *key = &KEY[i];
        current_level[i] = NEUQ_GPIO_read(KEY[i].KEY_PIN);
        if(current_level[i] == stable_level[i]){
            wave_cnt[i] = 0;
        }
        else{
             wave_cnt[i]++;        
             if (wave_cnt[i] >= KEY_CNT_MIN)
             {
                stable_level[i] = current_level[i];
                wave_cnt[i] = 0;
             }
        }
        if(stable_level[i] == GPIO_Low){
          
            if(key->KEY_PRESS_CNT < KEY_PRESS_LONG_CNT)
            {  
            key->KEY_PRESS_CNT++;
            if(key->KEY_PRESS_CNT == KEY_PRESS_LONG_CNT){
                    key->KEY_ACT = KEY_ACT_LONG;
                }
            }
        }
            else{
            if(key->KEY_PRESS_CNT >= KEY_PRESS_SHOART_CNT && key->KEY_PRESS_CNT < KEY_PRESS_LONG_CNT){
                key->KEY_ACT = KEY_ACT_SHORT;
            }
            key->KEY_PRESS_CNT = 0;
        }
    }
}
KEY_EventTypeDef KEY_GetEvent(void)
{
    uint8_t i;
    KEY_EventTypeDef event = KEY_EVENT_NONE;
    for(i = 0; i < KEY_NUM; i++){
        if(KEY[i].KEY_ACT == KEY_ACT_SHORT){
            event = (KEY_EventTypeDef)(i + 1);
            KEY[i].KEY_ACT = KEY_ACT_NONE;
            break;
        }else if(KEY[i].KEY_ACT == KEY_ACT_LONG){
            event = (KEY_EventTypeDef)(i + 5);
            KEY[i].KEY_ACT = KEY_ACT_NONE;  
            break;
          }
    }
    return event;
}