#ifndef KEY_H_
#define KEY_H_

#include "NEUQ_lib_include.h"

#define KEY_P_1  PE9  //方案一
#define KEY_P_2  PE8   //方案二
#define KEY_P_3  PE7   //方案三
#define KEY_P_4  PE10  //方案四 

#define KEY_NUM    4     //按键数量
#define KEY_ON_MS  20    //按键按下检测时间(ms)

typedef enum{
    KEY_EVENT_NONE = 0,   //无事件
    KEY_1,                //方案一
    KEY_2,                //方案二
    KEY_3,                //方案三
    KEY_4,                //方案四
}KEY_EVENT;

void Key_Init(void);           //按键初始化
void Key_Scan(void);           //按键扫描(每1ms调用一次)
KEY_EVENT Key_EventGet(void);  //获取按键状态

#endif