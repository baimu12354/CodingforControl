#ifndef KEY_INPUT_H
#define KEY_INPUT_H

/* K0 初始化与非阻塞消抖，只返回稳定按下沿。 */
#include <stdint.h>
void Key_Input_Init(void);
uint8_t Key_Input_Pressed(void);

#endif /* KEY_INPUT_H */
