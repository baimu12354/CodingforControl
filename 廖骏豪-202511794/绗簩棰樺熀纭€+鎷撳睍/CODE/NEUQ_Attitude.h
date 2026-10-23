/*
 * NEUQ_Attitude.h
 * 四元数姿态解算，Mahony互补滤波，输出roll/pitch/yaw
 *
 * 坐标轴（模块水平放、芯片丝印朝上，和MPU6050的芯片坐标系一致）
 *   X朝前 -> 绕X轴转是横滚roll，右倾为正
 *   Y朝右 -> 绕Y轴转是俯仰pitch，抬头为正
 *   Z朝上 -> 绕Z轴转是航向yaw，逆时针为正
 * 水平静止时acc约等于(0,0,1g)，三个角都接近0
 */
#ifndef _NEUQ_ATTITUDE_H_
#define _NEUQ_ATTITUDE_H_

#include "NEUQ_lib_include.h"

typedef struct{
    float q[4];             // q0(w) q1(x) q2(y) q3(z)，模长恒为1
    float roll;             // 横滚角，度
    float pitch;            // 俯仰角，度
    float yaw;              // 航向角，度，= yaw_raw - yaw_zero
    float yaw_raw;          // 没减零点的航向角
    float yaw_zero;         // 航向零位
    float kp;               // 比例增益
    float ki;               // 积分增益，六轴一般给0
    float dt_ms;            // 上次更新实际用的周期，ms
} NEUQ_Attitude_t;

extern NEUQ_Attitude_t g_att;

void NEUQ_Attitude_Init(void);

// 用一次加速度计采样直接把四元数摆到当前姿态
// 开机时模块不一定水平放着，从单位四元数开始让加速度计慢慢拉要好几秒才收敛，
// 那段时间屏幕上三个角一直在爬，看着像解算错了。静止时加速度计测的就是重力方向，
// 直接反推roll和pitch一步到位，比慢慢收敛靠谱。
void NEUQ_Attitude_InitFromAccel(const float acc_g[3]);

void NEUQ_Attitude_SetGains(float kp, float ki);
void NEUQ_Attitude_SetYawZero(void);
void NEUQ_Attitude_Update(const float acc_g[3], const float gyro_dps[3], float dt_s);

#endif /* _NEUQ_ATTITUDE_H_ */
