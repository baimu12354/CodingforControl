#ifndef NEUQ_KEY_H
#define NEUQ_KEY_H

#define KEY_UP PE9
#define KEY_DOWN PE8
#define KEY_FORWORD PE7
#define KEY_BACK PE10

#define KEY_NUM 4
#define KEY_SCAN_PERIOD_MS 10//扫描周期
#define KEY_SHORT_CNT      2 //短按计数
#define KEY_LONG_CNT       50//长按计数

typedef enum{
        KEY_ACT_NONE=0,      //无事件
        KEY_ACT_SHORT,       //短按
        KEY_ACT_LONG         //长按
}KEY_ACT_ENUM;

typedef enum{
    KEY_EVENT_NONE = 0,
    KEY_UP_SHORT,
    KEY_DOWN_SHORT,
    KEY_FORWORD_SHORT,
    KEY_BACK_SHORT,
    KEY_UP_LONG,
    KEY_DOWN_LONG,
    KEY_FORWORD_LONG,
    KEY_BACK_LONG
}KEY_EVENT_ENUM;

typedef struct{
     GPIO_Ports_ENUM Pin;
     uint16_t press_cnt;
     KEY_ACT_ENUM event;
}KEY_Typedef;

void NEUQ_Key_Init(void);
void NEUQ_Key_Scan(void);
KEY_EVENT_ENUM NEUQ_Key_Event(void);
#endif