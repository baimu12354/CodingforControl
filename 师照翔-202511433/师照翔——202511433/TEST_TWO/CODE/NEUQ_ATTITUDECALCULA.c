#include "NEUQ_lib_include.h"
#include <math.h>

// 四元数默认值
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// ===== PI 参数 =====
// Kp: 0.5 偏大，振动时抖；0.2~0.3 是 100Hz 采样的平衡点
#define twoKp       (2.0f * 0.30f)   // Kp = 0.30
#define twoKi       (2.0f * 0.002f)  // Ki = 0.002（很小，只为消残差）

static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;

// ==================== 常量 ====================
#define DT          0.01f
#define RAD_TO_DEG  57.29578f
#define DEG_TO_RAD  0.01745329f

// ===== 加速度有效范围（单位：g）=====
// 若 acandgy.x_accel 单位不是 g 而是 MPU6050 原始 LSB（±2g 量程 16384 LSB/g），
// 请改成：  ACC_MIN2 = 4096.0f*4096.0f， ACC_MAX2 = 49152.0f*49152.0f
#define ACC_MIN2    0.25f      // (0.5g)^2
#define ACC_MAX2    2.25f      // (1.5g)^2

// 陀螺仪零偏
extern float gyro_offset_x;
extern float gyro_offset_y;
extern float gyro_offset_z;

// 初始化
void Attitude_Init(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;

    attitude.roll  = 0.0f;
    attitude.pitch = 0.0f;
    attitude.yaw   = 0.0f;
}

// 姿态更新
void Attitude_Update(void)
{
    // 陀螺仪 deg/s -> rad/s，减零偏
    float gx = (acandgy.x_gyro - gyro_offset_x) * DEG_TO_RAD;
    float gy = (acandgy.y_gyro - gyro_offset_y) * DEG_TO_RAD;
    float gz = (acandgy.z_gyro - gyro_offset_z) * DEG_TO_RAD;

    float ax = acandgy.x_accel;
    float ay = acandgy.y_accel;
    float az = acandgy.z_accel;

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float accNorm2 = ax * ax + ay * ay + az * az;

    // 只在加速度模长合理时才用加速度修正（排除运动/碰撞/通信异常）
    if ((accNorm2 > ACC_MIN2) && (accNorm2 < ACC_MAX2))
    {
        // 2.1 归一化
        recipNorm = 1.0f / sqrtf(accNorm2);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 2.2 四元数估计重力方向（机体坐标系）
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 2.3 叉积误差
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 2.4 积分反馈（带限幅，防止积分饱和）
        integralFBx += twoKi * halfex * DT;
        integralFBy += twoKi * halfey * DT;
        integralFBz += twoKi * halfez * DT;

        if (integralFBx >  0.5f) integralFBx =  0.5f;
        if (integralFBx < -0.5f) integralFBx = -0.5f;
        if (integralFBy >  0.5f) integralFBy =  0.5f;
        if (integralFBy < -0.5f) integralFBy = -0.5f;
        if (integralFBz >  0.5f) integralFBz =  0.5f;
        if (integralFBz < -0.5f) integralFBz = -0.5f;

        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;

        // 2.5 比例反馈
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 3. 一阶龙格库塔积分
    gx *= (0.5f * DT);
    gy *= (0.5f * DT);
    gz *= (0.5f * DT);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // 4. 归一化
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // 5. 四元数转欧拉角（关键：asinf 参数必须 clamp，否则会出 NaN -> 显示乱跳）
    float sinp = 2.0f * (q0 * q2 - q1 * q3);   // 等价于 -2*(q1*q3 - q0*q2)
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    attitude.roll  = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * RAD_TO_DEG;
    attitude.pitch = asinf(sinp) * RAD_TO_DEG;
    attitude.yaw   = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * RAD_TO_DEG;
}