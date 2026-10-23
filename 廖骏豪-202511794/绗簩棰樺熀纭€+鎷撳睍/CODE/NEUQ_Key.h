/*
 * NEUQ_Key.h
 * 四个按键的扫描、消抖、短按长按识别
 *
 * 接线：
 *   KEY_FORWARD PE7   进页面 / 开始编辑 / 确认
 *   KEY_DOWN    PE8   下一项 / 参数减
 *   KEY_BACK    PE9   返回上一级 / 取消编辑
 *   KEY_UP      PE10  上一项 / 参数加
 * 按键一端接IO一端接GND，所以配上拉输入，按下是低电平。
 *
 * 如果引脚接得和这里不一致，按键就会"串味"：
 * 比如OK其实接在PE10上，那在设置页按OK只会让光标上移，怎么按都进不了编辑。
 * 碰到"按键行为不对"先核对下面四个宏。
 *
 * 处理约定（每10ms调一次NEUQ_Key_Scan）：
 *   1. 消抖：连着2次（20ms）都是低电平才算按下
 *   2. 短按：松手时产生一次SHORT事件
 *   3. 长按连发：只有UP/DOWN支持。按住350ms报一次LONG，之后每50ms再报一次
 *   4. OK/BACK不做长按，无论按多久松手都只报一次SHORT
 *      因为让长按和短按做两件不同的事很容易误触发，
 *      手重一点就可能"想进编辑结果触发了保存"。
 *      所以KEY_FORWARD_LONG和KEY_BACK_LONG这两个事件永远不会出现。
 */
#ifndef _NEUQ_KEY_H
#define _NEUQ_KEY_H

#include "NEUQ_lib_include.h"

// 按键引脚
#define KEY_UP          PE10
#define KEY_DOWN        PE8
#define KEY_FORWARD     PE7
#define KEY_BACK        PE9

#define KEY_NUM                 (4)
#define KEY_SCAN_PERIOD_MS      (10)    // 扫描周期，NEUQ_Key_Scan必须按这个周期调
#define KEY_SHORT_CNT           (2)     // 2*10ms=20ms消抖
#define KEY_LONG_CNT            (35)    // 35*10ms=350ms算长按，只对UP/DOWN有效
#define KEY_REPEAT_CNT          (5)     // 长按之后每5*10ms=50ms重复一次

#define KEY_LONG_REPEAT_NUM     (2)     // 前2个（UP/DOWN）支持长按连发

// 单个按键的瞬时动作
typedef enum{
    KEY_ACT_NONE = 0,
    KEY_ACT_SHORT,
    KEY_ACT_LONG
} KEY_ACT_ENUM;

// 送给应用层的按键事件
typedef enum{
    KEY_EVENT_NONE = 0,
    KEY_UP_SHORT,
    KEY_UP_LONG,
    KEY_DOWN_SHORT,
    KEY_DOWN_LONG,
    KEY_FORWARD_SHORT,
    KEY_FORWARD_LONG,
    KEY_BACK_SHORT,
    KEY_BACK_LONG
} KEY_EVENT_ENUM;

void NEUQ_Key_Init(void);
void NEUQ_Key_Scan(void);
KEY_EVENT_ENUM NEUQ_Key_GetEvent(void);

#endif /* _NEUQ_KEY_H */
