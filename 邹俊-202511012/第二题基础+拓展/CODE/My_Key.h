/*********************************************************************************************************************
* 文件名称          My_Key
* 功能说明          菜单按键（上 / 下 / 确认 / 返回）：定时中断里扫描消抖，主循环里取事件
*
*                   为什么这样分工：
*                   1. 扫描放在定时中断里，每 10 ms 固定采样一次，消抖时间准确，主循环再忙也不会漏键
*                   2. 中断里只"记下发生了什么按键事件"，菜单处理放到主循环，中断保持很短，
*                      不会拖慢同一个中断里的 LED 时序 / 传感器采样
*
*                   使用方法：
*                   My_Key_init();                          // 初始化引脚
*                   My_Key_scan();                          // 在定时中断里每 KEY_SCAN_PERIOD_MS 调用一次
*                   while (My_Key_get_event(&event)) {...}  // 主循环里取出所有事件逐个处理
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#ifndef _MY_KEY_H_
#define _MY_KEY_H_

#include <stdint.h>

typedef enum {
    KEY_UP = 0,                 // 上 / 数值增大
    KEY_DOWN,                   // 下 / 数值减小
    KEY_OK,                     // 确认 / 进入
    KEY_BACK,                   // 返回 / 取消
    KEY_NUM                     // 按键个数
} Key_Id_Enum;

typedef enum {
    KEY_EVENT_PRESS  = 1,       // 按下（消抖后只产生一次）
    KEY_EVENT_REPEAT = 2,       // 按住不放时的连发（只有 上 / 下 键有，用于快速移动光标、快速加减数值）
} Key_Event_Type_Enum;

typedef struct {
    uint8_t id;                 // 哪个键，见 Key_Id_Enum
    uint8_t type;               // 什么事件，见 Key_Event_Type_Enum
} Key_Event_Struct;

void    My_Key_init(void);
void    My_Key_scan(void);                         // 定时中断中调用
uint8_t My_Key_get_event(Key_Event_Struct *event); // 主循环中调用，取到事件返回 1，没有事件返回 0
void    My_Key_clear_events(void);                 // 丢弃还没处理的事件（例如开机过程中误按的键）

#endif
