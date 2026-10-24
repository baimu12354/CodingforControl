#ifndef _KEY_
#define _KEY_
#include "NEUQ_GPIO.h"
#define KEY_UP          PE9
#define KEY_DOWN        PE8
#define KEY_FORWARD     PE7
#define KEY_BACK        PE10

#define KEY_NUM           4
/*key判定要用到的参数的定义*/
#define KEY_SCAN_PERIOD   10   
#define KEY_SHORT_CNT     2    
#define KEY_LONG_CNT      50   

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
    GPIO_Ports_ENUM pin;        /* 该按键占用的引脚，类型就是 GPIO 端口枚举，可直接喂给 NEUQ_GPIO_read */
    int16_t         press_cnt;  /* 按下期间经过的扫描次数，用于消抖和判定长短按，只有中断里碰它 */
    volatile KEY_ACT_ENUM event;/* 待主循环取走的事件，取走前一直保持。
                                 * 中断里置位、主循环读取并清零，跨上下文共享，必须加 volatile */
}KEY_Typedef;


void NEUQ_Key_Init(void);
void NEUQ_Key_Scan(void);
KEY_EVENT_ENUM NEUQ_Key_EventGet(void);

#endif