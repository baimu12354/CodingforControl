#ifndef _NEUQ_KEY_H_
#define _NEUQ_KEY_H_

#include "NEUQ_lib_include.h"

#define NEUQ_KEY_PIN (PE9)  //按键引脚

#define NEUQ_KEY_READ() (NEUQ_GPIO_read(NEUQ_KEY_PIN) == GPIO_Low ? 1 : 0)  //按键读取，1按下，0松开

void NEUQ_Key_init(void);

uint8_t NEUQ_Key_scan(void);

#endif