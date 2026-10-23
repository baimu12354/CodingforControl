#ifndef KEY_INPUT_H
#define KEY_INPUT_H

/* 四个按键的初始化、非阻塞消抖和按下事件位图。 */
#include <stdint.h>
void Key_Input_Init(void);
uint8_t Key_Input_ReadEvents(void);

#endif /* KEY_INPUT_H */
