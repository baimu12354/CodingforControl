#ifndef _NEUQ_KEY_H
#define _NEUQ_KEY_H
#ifndef NEUQ_MENU_SIM          // PC 仿真时不需要真库，仿真文件里会提供同名的假 GPIO
#include "NEUQ_GPIO.h"         // 只用到引脚枚举（PE7~PE10）和 GPIO 读写接口。
                               // 千万不要在这里 include "NEUQ_lib_include.h"：
                               // 那个头文件又会 include 回 NEUQ_Menu.h，而 NEUQ_Menu.h
                               // 需要本文件下面的 KEY_EVENT_ENUM，形成循环包含，
                               // 单独编译 NEUQ_Key.c 时会报 "KEY_EVENT_ENUM is undefined"。
#endif
#define KEY_UP          PE9
#define KEY_DOWN        PE8
#define KEY_FORWARD     PE7
#define KEY_BACK        PE10

#define KEY_NUM           4

#define KEY_SCAN_PERIOD   10   // 扫描周期10ms
#define KEY_SHORT_CNT     2    // 短按阈值：2*10 = 20ms（用于消抖和判定短按）
#define KEY_LONG_CNT      50   // 长按阈值：50*10 = 500ms
#define KEY_REPEAT_CNT    10   // 长按后的重复间隔：10*10 = 100ms（一直按着就连续加减）

typedef enum{
    KEY_ACT_NONE = 0,
    KEY_ACT_SHORT,
    KEY_ACT_LONG
}KEY_ACT_ENUM;

typedef enum{
    KEY_EVENT_NONE = 0,
    KEY_UP_SHORT,      // 1
    KEY_DOWN_SHORT,    // 2
    KEY_FORWARD_SHORT, // 3
    KEY_BACK_SHORT,    // 4
    KEY_UP_LONG,       // 5
    KEY_DOWN_LONG,     // 6
    KEY_FORWARD_LONG,  // 7
    KEY_BACK_LONG      // 8
}KEY_EVENT_ENUM;

typedef struct {
    int16_t        pin;         // 引脚编号（就是 GPIO_Ports_ENUM 的值）
    int16_t        press_cnt;   // 已经按住了多少个扫描周期
    int16_t        repeat_cnt;  // 进入长按后，用来数"该重复了没有"
    KEY_ACT_ENUM   event;       // 本次待取走的事件（注意类型是 ACT，不是 EVENT）
}KEY_Typedef;


void NEUQ_Key_Init(void);
void NEUQ_Key_Scan(void);
KEY_EVENT_ENUM NEUQ_Key_EventGet(void);

#endif
