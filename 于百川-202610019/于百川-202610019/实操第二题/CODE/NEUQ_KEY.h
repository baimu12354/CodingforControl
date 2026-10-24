#ifndef _NEUQ_KEY_H_
#define _NEUQ_KEY_H_

#include "NEUQ_lib_include.h"


#define KEY_UP_PIN       PE10
#define KEY_DOWN_PIN     PE8
#define KEY_FORWARD_PIN  PE7
#define KEY_BACK_PIN     PE9

#define KEY_NUM          4

#define KEY_PRESS_LEVEL  GPIO_Low   // 上拉输入：按下为低（低有效）；若你的板子高有效，改成 GPIO_High 
#define KEY_DEBOUNCE_MS  10         // 消抖时间(ms)，Scan按1ms节拍调用 


//按键事件枚举 
typedef enum{
    KEY_NONE = 0,   /* 无按键 */
    KEY_UP,
    KEY_DOWN,
    KEY_FORWARD,
    KEY_BACK
}KEY_EVENT_ENUM;

void NEUQ_KEY_Init(void);            // 初始化
void NEUQ_KEY_Scan(void);            // 1ms节拍扫描
KEY_EVENT_ENUM NEUQ_KEY_Get(void);   // 取按键事件

#endif