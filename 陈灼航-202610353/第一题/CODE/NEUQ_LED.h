#ifndef _NEUQ_LED_H_
#define _NEUQ_LED_H_

#include "NEUQ_lib_include.h"

#define LED_PIN        PA8    //LED 控制引脚（推挽输出，高电平亮）
#define LED_SCHEME_NUM 4      //方案总数
#define LED_TICK_MS    10     //状态机推进周期(ms)，和 PIT 中断周期一致

//方案编号
typedef enum{
    LED_SCHEME_1 = 1, //方案1
    LED_SCHEME_2,     //方案2
    LED_SCHEME_3,     //方案3
    LED_SCHEME_4,     //方案4
}LED_Scheme;

void NEUQ_LED_Init(void);                 //LED 初始化
void NEUQ_LED_Tick(void);                 //状态机推进
void NEUQ_LED_SetPending(uint8_t scheme); //设置待执行方案
uint8_t NEUQ_LED_GetCurrent(void);        //读当前正在执行的方案
uint8_t NEUQ_LED_GetPending(void);        //读待执行方案

#endif
