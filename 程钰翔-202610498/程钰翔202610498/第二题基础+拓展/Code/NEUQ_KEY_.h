#ifndef _NEUQ_KEY_H_
#define _NEUQ_KEY_H_

#include "NEUQ_lib_include.h"

#define KEY_UP_PIN     PE10
#define KEY_DOWN_PIN   PE8
#define KEY_BACK_PIN   PE7
#define KEY_IN_PIN     PE9
#define KEY_NUM  4

extern volatile uint8_t  current_key_id;   //当前的按键编码
extern volatile uint32_t g_key_tick;       //1ms 系统节拍，由 PIT1 中断累加

typedef enum{
    EVENT_NULL=0,         //无事件 0
    EVENT_KEY_UP,         //1
    EVENT_KEY_DOWN,       //2
    EVENT_KEY_BACK,       //3
    EVENT_KEY_IN          //4   便于之后赋值给current_key_id

}KeyEvent_t;


typedef struct 
{
    GPIO_Ports_ENUM  Port;
    GPIO_Level       Last_level;        // 已确认的稳定电平（消抖后）
    KeyEvent_t       Press_event;
    uint32_t         Last_change_tick;  // 最近一次读到稳定电平的时刻(ms)
}key_type;


void  NEUQ_KEY_Init(key_type*key,GPIO_Ports_ENUM pin,KeyEvent_t Press_event);
void NEUQ_KEY_Scan(key_type*key);
void NEUQ_KEY_ALL_SCAN(void);
void NEUQ_KEY_ALL_Init(void);



#endif







