#ifndef NEUQ_ATT_H_
#define NEUQ_ATT_H_

#include "stm32f4xx_hal.h"

/* 常用单位换算 */
#define DEG2RAD   (0.0174533f)    /* 度  → 弧度 */
#define ATT_RAD2DEG   (57.29578f)     /* 弧度 → 度  */

typedef struct
{
    /* 四元数 */
    float q0;               /* w 分量 */
    float q1;               /* x 分量 */
    float q2;               /* y 分量 */
    float q3;               /* z 分量 */

    /* PI 增益 */
    float Kp;
    float Ki;

    /* 积分反馈项 */
    float integral_x;
    float integral_y;
    float integral_z;

    /* 输出欧拉角（弧度） */
    float roll;             /* 横滚：绕 X 轴 */
    float pitch;            /* 俯仰：绕 Y 轴 */
    float yaw;              /* 偏航：绕 Z 轴 */

    /* 陀螺仪零偏（rad/s） */
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
}Attitude_t;

extern Attitude_t att;

void Attitude_Init(void);
void Attitude_CalibrateGyro(void);
void Attitude_Update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt);




#endif