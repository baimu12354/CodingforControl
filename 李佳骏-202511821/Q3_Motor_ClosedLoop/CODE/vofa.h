/* vofa.h - VOFA+ 接口 */

#ifndef _VOFA_H_
#define _VOFA_H_

#include "stm32f4xx_hal.h"

/* JustFloat：小端 float + 帧尾 00 00 80 7F
 * 通道：目标、实际、Kp、Ki、Kd、运行、PWM、编码器计数、接受数、拒绝数 */
void Vofa_SendFloats(const float *values, uint8_t count);

#endif /* _VOFA_H_ */
