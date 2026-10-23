#include "mahony.h"
#include <math.h>

// 姿态角 全局变量
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;


// 四元数
static float q1 = 1.0f, q2 = 0.0f, q3 = 0.0f, q4 = 0.0f;
// 互补滤波参数
static float Kp = 2.0f;
static float Ki = 0.005f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MahonyUpdate(float ax,float ay,float az,float gx,float gy,float gz,float dt)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, by;
    float vx, vy, vz;
    float ex, ey, ez;
    float iq1, iq2, iq3, iq4;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        vx = 2.0f * q2*q4 - 2.0f * q1*q3;
        vy = 2.0f * q1*q2 + 2.0f * q3*q4;
        vz = q1*q1 - q2*q2 - q3*q3 + q4*q4;

        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        integralFBx += ex * Ki * dt;
        integralFBy += ey * Ki * dt;
        integralFBz += ez * Ki * dt;

        gx += Kp*ex + integralFBx;
        gy += Kp*ey + integralFBy;
        gz += Kp*ez + integralFBz;
    }

    q1 += (-q2*gx - q3*gy - q4*gz)*0.5f * dt;
    q2 += ( q1*gx + q3*gz - q4*gy)*0.5f * dt;
    q3 += ( q1*gy - q2*gz + q4*gx)*0.5f * dt;
    q4 += ( q1*gz + q2*gy - q3*gx)*0.5f * dt;

    recipNorm = invSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    q4 *= recipNorm;

    // 四元数转欧拉角(角度)
    roll = atan2f(q1*q2 + q3*q4, 0.5f - q2*q2 - q3*q3) * 57.29578f;
    pitch = asinf(-2.0f * (q2*q4 - q1*q3)) * 57.29578f;
    yaw = atan2f(q2*q3 + q1*q4, 0.5f - q3*q3 - q4*q4) * 57.29578f;
}