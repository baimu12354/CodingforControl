


#ifndef __NEUQ_MAHONY_H
#define __NEUQ_MAHONY_H

#include "stm32f4xx_hal.h"

// 姿态角，单位 度
extern float roll;
extern float pitch;
extern float yaw;

// Mahony解算函数
// ax,ay,az：加速度(g)；gx,gy,gz：陀螺仪(rad/s)；dt：采样周期(秒)
void MahonyUpdate(float ax,float ay,float az,float gx,float gy,float gz,float dt);




#endif 