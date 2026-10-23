#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include <stdint.h>

// 姿态解算模块：陀螺零偏校准 + Madgwick 四元数 -> roll/pitch/yaw
// 主循环里调用一次 Attitude_Update() 即可，后台一直运行。

void     Attitude_Update(float gx, float gy, float gz,   // 陀螺 deg/s
                         float ax, float ay, float az);  // 加速度 g
uint8_t  Attitude_Calibrated(void);
float    Attitude_GetRoll(void);
float    Attitude_GetPitch(void);
float    Attitude_GetYaw(void);

#endif  /* __ATTITUDE_H__ */
