#include "NEUQ_lib_include.h"

static const GPIO_Ports_ENUM keyPins[KEY_NUM]={
    PE7,
    PE8
};

static const KeyEventEnum keyEvents[KEY_NUM]={
    KEY_UP_SHORT,
    KEY_DOWN_SHORT
};

typedef enum{
    KEY_STATE_IDLE,
    KEY_STATE_DEBOUNCE,
    KEY_STATE_PRESSED
}Keystate;

static Keystate key_state[KEY_NUM];
static uint16_t key_press_cnt[KEY_NUM];
static KeyEventEnum key_event[KEY_NUM];

void NEUQ_Key_Init(void){
    for (uint8_t i=0;i<KEY_NUM;i++){
        NEUQ_GPIO_init(keyPins[i],GPIO_Mode_Input_PullUp,GPIO_High);
        key_state[i]=KEY_STATE_IDLE;
        key_press_cnt[i]=0;
        key_event[i]=KEY_EVENT_NONE;
    }
}

void NEUQ_Key_Scan(void){
    for(uint8_t i=0;i<KEY_NUM;i++){
      uint8_t is_pressed = (NEUQ_GPIO_read(keyPins[i])==GPIO_Low);  
      switch(key_state[i]){
        case KEY_STATE_IDLE:
        if( is_pressed ){
          key_state[i] =  KEY_STATE_DEBOUNCE;
          key_press_cnt[i]=0;
        }
        break;
        case KEY_STATE_DEBOUNCE:
        if( is_pressed ){
          key_press_cnt[i]++;
          if(key_press_cnt[i]>=2)  {
            key_state[i] =  KEY_STATE_PRESSED;
            key_press_cnt[i]=0;
          }
        }
          else{
            key_state[i] =  KEY_STATE_IDLE;
          }
        break;
        case KEY_STATE_PRESSED:
        if( is_pressed ){
          key_press_cnt[i]++;
        }else{
            key_event[i]=keyEvents[i];
            key_state[i]=KEY_STATE_IDLE;
        }
        break;
      }
    }
}
KeyEventEnum NEUQ_Key_EventGet(void){
    for(uint8_t i=0;i<KEY_NUM;i++){
        if(key_event[i]!=KEY_EVENT_NONE){
            KeyEventEnum evt = key_event[i];
            key_event[i]=KEY_EVENT_NONE;
            return evt;
        }
    }
    return KEY_EVENT_NONE; 
}
