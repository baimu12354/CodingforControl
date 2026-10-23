#ifndef KEY_H_
#define KEY_H_

#include "NEUQ_lib_include.h"

#define KEY_UP      PE9   //上键
#define KEY_DOWN    PE8   //下键
#define KEY_FORWARD PE7   //进入键/确认键
#define KEY_BACK    PE10  //返回键/取消键

#define KEY_NUM      4     //按键数量
#define KEY_SCAN_T   10    //按键检测周期10ms
#define KEY_S_PRESS  2     //短按检测2周期
#define KEY_L_PRESS  50    //长按检测50周期
#define KEY_L_REPEAT 5     //长按自动重复5周期

typedef enum{
    KEY_STATE_NONE = 0,   //按键未按下
    KEY_STATE_S,          //短按
    KEY_STATE_L,          //长按
}KEY_STATE;  //按键状态

typedef enum{
    KEY_EVENT_NONE = 0,
    KEY_S_UP,
    KEY_S_DOWN,
    KEY_S_FORWARD,
    KEY_S_BACK,
    KEY_L_UP,
    KEY_L_DOWN,
    KEY_L_FORWARD,
    KEY_L_BACK,
}KEY_EVENT;   //按键具体动作

void Key_Init(void);           //按键初始化
void Key_Scan(void);           //按键扫描
KEY_EVENT Key_EventGet(void);  //获取按键具体动作

#endif