#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include <stdint.h>

// ��̬����ģ�飺�����˲�(�����ǻ��� + ���ٶȼƽǶ�) -> roll/pitch/yaw
// ��ѭ��ÿ�ε���һ�� Attitude_Update() ���ɣ���̨һֱ���С�

void     Attitude_Update(float gx, float gy, float gz,   // ������ deg/s
                         float ax, float ay, float az);  // ���ٶȼ� g
uint8_t  Attitude_Calibrated(void);
float    Attitude_GetRoll(void);
float    Attitude_GetPitch(void);
float    Attitude_GetYaw(void);
void Attitude_Calib(void);
#endif  /* __ATTITUDE_H__ */
