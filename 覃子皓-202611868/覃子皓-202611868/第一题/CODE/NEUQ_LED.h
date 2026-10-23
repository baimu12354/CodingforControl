#ifndef _NEUQ_LED_H_
#define _NEUQ_LED_H_

#include "NEUQ_lib_include.h"

#define NEUQ_LED_PIN (PA8)  //led引脚

void NEUQ_LED_init(void);  //初始化
void NEUQ_LED_on(void);   //开
void NEUQ_LED_off(void);   //关


#endif