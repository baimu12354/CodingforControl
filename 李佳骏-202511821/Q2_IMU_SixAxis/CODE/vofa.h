/* vofa.h - VOFA+ 接口 */

#ifndef _VOFA_H_
#define _VOFA_H_

#include "stm32f4xx_hal.h"

/* 发送 count 个浮点通道，函数内部自动补帧尾 */
void Vofa_SendFloats(const float *values, uint8_t count);

#endif /* _VOFA_H_ */
