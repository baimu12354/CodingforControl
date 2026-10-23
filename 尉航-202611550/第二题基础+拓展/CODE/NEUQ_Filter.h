#ifndef __NEUQ_FILTER_H__
#define __NEUQ_FILTER_H__

#include "NEUQ_lib_include.h"

// 1维卡尔曼
typedef struct {
    float x;     // 状态
    float A;     // x(n)=A*x(n-1)+u(n)
    float H;     // z(n)=H*x(n)+w(n)
    float q;     // 过程噪声方差
    float r;     // 测量噪声方差
    float p;     // 估计误差协方差
    float gain;  // 卡尔曼增益
} kalman_struct;

extern float raw[6];
extern float filtered[6];
extern kalman_struct kf[6];
extern float roll_angle;
extern float pitch_angle;
extern float yaw_angle;
extern float gyro_bias_deg[3];
void kalman_init(kalman_struct *k, float init_x, float init_p);
float kalman_filter(kalman_struct *k, float measure);
void Filter_SixAxis(void);
void Gyro_Bias_Calibration(void);
void Filter_ApplyParams(float q, float r);
void Quat_Update(float gx, float gy, float gz,float ax, float ay, float az, float dt);
void Quat_ToEuler(void);

#endif
