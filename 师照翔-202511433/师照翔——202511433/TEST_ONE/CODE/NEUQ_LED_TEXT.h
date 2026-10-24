#ifndef LED_TEST_H_
#define LED_TEST_H_

#include<stdint.h>

extern uint32_t stratage_req;//将要运行的方案代号
extern uint32_t start_tick;
extern uint8_t is_finish;       //标记该次方案的一次循环是否完成，0->未完成、1->完成
extern uint32_t stratage_run;     //正在执行的代码编号
extern uint8_t step; 

void LED_Stratage1(void);
void LED_Stratage2(void);
void LED_Stratage3(void);
void LED_Stratage4(void);
uint32_t NEUQ_Count_ms(void);

#endif