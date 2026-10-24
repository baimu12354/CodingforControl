//=============================================================================================
// MadgwickAHRS.c
//=============================================================================================
//
// Madgwick 六轴 IMU 姿态解算（已从 C++ 版改成纯 C 版）。
// 原始算法：http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/ （GPL 许可）
//
// 与原始 C++ 版的差异：
//   1. 类(class) 改成模块级 static 变量 + 一组函数；
//   2. 角速度输入直接是 rad/s（原始库收 °/s 再内部换算，这里省去那步，算法本身就要 rad/s）；
//   3. invSqrt 用 1.0f/sqrtf()，不用“快速倒数平方根”位运算（更安全、可移植）；
//   4. asinf 前对参数夹紧到 [-1,1]，避免浮点误差导致 NaN；
//   5. 只保留 6 轴版本（本工程 MPU6050 无磁力计，9 轴 update 用不到）。
//=============================================================================================
#include "MadgwickAHRS.h"
#include <math.h>

#define RAD_TO_DEG  57.29578f      // 180/π（与原始库一致）

// ---- 模块状态（原 C++ 类的成员变量）----
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // 四元数
static float beta = 0.1f;                                    // 算法增益
static float invSampleFreq = 1.0f / 512.0f;                  // 采样周期(秒)
static float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;          // 欧拉角(弧度)
static char  anglesComputed = 0;

// ---- 内部函数 ----
static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static void computeAngles(void)
{
    float sp = -2.0f * (q1 * q3 - q0 * q2);
    if (sp >  1.0f) sp =  1.0f;      // 防浮点误差越界导致 asinf 返回 NaN
    if (sp < -1.0f) sp = -1.0f;

    roll  = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
    pitch = asinf(sp);
    yaw   = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
    anglesComputed = 1;
}

// ---- 对外接口 ----
void Madgwick_Init(float sampleFrequency)
{
    invSampleFreq = 1.0f / sampleFrequency;
    beta = 0.1f;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    anglesComputed = 0;
}

void Madgwick_SetBeta(float b)
{
    beta = b;
}

void Madgwick_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 1) 由陀螺仪积分算四元数变化率（gx/gy/gz 已是 rad/s）
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // 2) 加速度计读数有效时，用“重力方向”做梯度下降修正
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;   // 加速度归一化

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q2 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // 3) 积分：变化率 × 采样周期
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    // 4) 四元数归一化（保证模长恒为 1）
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    anglesComputed = 0;   // 姿态已更新，下次取角时重新换算
}

float Madgwick_GetRoll(void)  { if (!anglesComputed) computeAngles(); return roll  * RAD_TO_DEG; }
float Madgwick_GetPitch(void) { if (!anglesComputed) computeAngles(); return pitch * RAD_TO_DEG; }
float Madgwick_GetYaw(void)   { if (!anglesComputed) computeAngles(); return yaw   * RAD_TO_DEG + 180.0f; }
float Madgwick_GetRollRadians(void)  { if (!anglesComputed) computeAngles(); return roll; }
float Madgwick_GetPitchRadians(void) { if (!anglesComputed) computeAngles(); return pitch; }
float Madgwick_GetYawRadians(void)   { if (!anglesComputed) computeAngles(); return yaw; }
