#ifndef _MOTOR_SPEED_H_
#define _MOTOR_SPEED_H_

#include "app_common.h"

void  Speed_Init(void);                       /* 初始化编码器基准计数值 */
void  Speed_Accum1ms(void);                   /* 1ms: 编码器计数差入环形缓冲 */
float Speed_Calc(float target, float actual); /* 节拍到时: 自适应双窗 + 低通, 返回滤波后转速 */

#endif /* _MOTOR_SPEED_H_ */
