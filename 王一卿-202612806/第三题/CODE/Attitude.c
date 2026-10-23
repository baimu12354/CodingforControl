#include "NEUQ_lib_include.h"
#include "Attitude.h"
#include <math.h>

#define MADGWICK_BETA     0.1f
#define GYRO_CAL_SAMPLES  400
#define DEG2RAD           0.01745329252f
#define RAD2DEG           57.29577951f

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // 姿态四元数
static float att_roll = 0.0f, att_pitch = 0.0f, att_yaw = 0.0f;

static uint8_t  calibrated = 0;          // 陀螺零偏是否标定完成
static uint32_t cal_count  = 0;
static float    cal_gx = 0.0f, cal_gy = 0.0f, cal_gz = 0.0f;
static float    bias_gx = 0.0f, bias_gy = 0.0f, bias_gz = 0.0f;
static uint32_t last_tick = 0;           // 上次解算时刻（NEUQ 计时器微秒值）

// 上电静止采样求零偏 -> 之后每次调用做一步 Madgwick 六轴解算，输出 roll/pitch/yaw
void Attitude_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    if (calibrated == 0) {                 // 零偏校准
        cal_gx += gx; cal_gy += gy; cal_gz += gz;
        if (++cal_count >= GYRO_CAL_SAMPLES) {
            bias_gx = cal_gx / (float)cal_count;
            bias_gy = cal_gy / (float)cal_count;
            bias_gz = cal_gz / (float)cal_count;
            calibrated = 1;
            last_tick = NEUQ_Get_Timer_Value_us();
        }
        return;
    }

    uint32_t now = NEUQ_Get_Timer_Value_us();
    uint32_t dt_us = (uint32_t)(uint16_t)(now - last_tick);   // 距上次的微秒，自动处理 65ms 回绕
    float dt = (float)dt_us / 1000000.0f;                      // 微秒 -> 秒
    last_tick = now;
    if (dt <= 0.0f || dt >= 0.1f) return;  // 剔除计数器回绕/异常 dt

    gx = (gx - bias_gx) * DEG2RAD;
    gy = (gy - bias_gy) * DEG2RAD;
    gz = (gz - bias_gz) * DEG2RAD;

    // Madgwick：陀螺积分 + 加速度计梯度下降修正
    float qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        float recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
        float _4q0 = 4.0f*q0, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2;
        float _8q1 = 8.0f*q1, _8q2 = 8.0f*q2;
        float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;

        float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        float s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        float s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

        recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= MADGWICK_BETA*s0; qDot2 -= MADGWICK_BETA*s1;
        qDot3 -= MADGWICK_BETA*s2; qDot4 -= MADGWICK_BETA*s3;
    }

    q0 += qDot1*dt; q1 += qDot2*dt; q2 += qDot3*dt; q3 += qDot4*dt;
    float recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    // 四元数 -> 欧拉角(度)
    att_roll  = RAD2DEG * atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    att_pitch = RAD2DEG * asinf(2.0f*(q0*q2 - q3*q1));
    att_yaw   = RAD2DEG * atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}

uint8_t Attitude_Calibrated(void) { return calibrated; }
float   Attitude_GetRoll(void)    { return att_roll; }
float   Attitude_GetPitch(void)   { return att_pitch; }
float   Attitude_GetYaw(void)     { return att_yaw; }
