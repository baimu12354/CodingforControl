/*********************************************************************************************************************
* 文件名称          Task2_Attitude
* 功能说明          第二题挑战部分：Mahony 四元数姿态解算 + 陀螺仪零偏校准（实现），说明见 Task2_Attitude.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#include <math.h>
#include "Task_Config.h"
#include "Task2_Attitude.h"

#define RAD_TO_DEG          (57.29577951f)
#define CALIB_SAMPLES       (T2_CALIB_TIME_MS / T2_SAMPLE_PERIOD_MS)    // 校准需要的样本数（默认 2000/5 = 400）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数归一化：让 q0²+q1²+q2²+q3² = 1
// 备注信息     每次积分都会引入一点数值误差，不归一化的话四元数的模会慢慢偏离 1，不再表示一个纯旋转
//-------------------------------------------------------------------------------------------------------------------
static void att_normalize(T2_Attitude_Struct *a)
{
    float norm = sqrtf(a->q0 * a->q0 + a->q1 * a->q1 + a->q2 * a->q2 + a->q3 * a->q3);
    if (norm < 1e-9f) {                     // 异常情况：恢复成"没有旋转"
        a->q0 = 1.0f;
        a->q1 = a->q2 = a->q3 = 0.0f;
        return;
    }
    a->q0 /= norm;
    a->q1 /= norm;
    a->q2 /= norm;
    a->q3 /= norm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数 -> 欧拉角（ZYX 顺序），单位度
//-------------------------------------------------------------------------------------------------------------------
static void att_to_euler(T2_Attitude_Struct *a)
{
    float q0 = a->q0, q1 = a->q1, q2 = a->q2, q3 = a->q3;
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);

    if (sin_pitch > 1.0f)  sin_pitch = 1.0f;    // 数值误差可能让它略超出 ±1，asin 会得到 NaN
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    a->roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    a->pitch = asinf(sin_pitch) * RAD_TO_DEG;
    a->yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     欧拉角（弧度）-> 四元数
//-------------------------------------------------------------------------------------------------------------------
static void att_from_euler(T2_Attitude_Struct *a, float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f),   sy = sinf(yaw * 0.5f);

    a->q0 = cr * cp * cy + sr * sp * sy;
    a->q1 = sr * cp * cy - cr * sp * sy;
    a->q2 = cr * sp * cy + sr * cp * sy;
    a->q3 = cr * cp * sy - sr * sp * cy;
    att_normalize(a);
    att_to_euler(a);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用静止时的加速度（也就是重力方向）直接算出横滚、俯仰，作为初始姿态；航向置 0
// 备注信息     静止时加速度计只测到重力：roll = atan2(ay, az)，pitch = atan2(-ax, sqrt(ay² + az²))
//              这样校准一结束姿态就是对的，不需要等滤波器慢慢收敛
//-------------------------------------------------------------------------------------------------------------------
void T2_Attitude_reset(T2_Attitude_Struct *att, float ax, float ay, float az)
{
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    att_from_euler(att, roll, pitch, 0.0f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Mahony 姿态更新一次（每个采样周期调用一次）
// 参数说明     gx gy gz    去掉零偏后的角速度，单位 rad/s
// 参数说明     ax ay az    加速度，单位 g
// 参数说明     dt          采样周期，单位 s
//-------------------------------------------------------------------------------------------------------------------
void T2_Attitude_update(T2_Attitude_Struct *att, float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    float q0 = att->q0, q1 = att->q1, q2 = att->q2, q3 = att->q3;
    float norm = sqrtf(ax * ax + ay * ay + az * az);

    // 第 1 步：用重力方向修正角速度
    // 只有加速度模长接近 1 g（0.7 ~ 1.3 g）时才修正：模长偏离太多说明模块正在被甩动，
    // 这时加速度计测到的不只是重力，用它修正反而会出错
    if (norm > 0.7f && norm < 1.3f) {
        float vx, vy, vz, ex, ey, ez;
        ax /= norm;                         // 测得的重力方向（单位向量）
        ay /= norm;
        az /= norm;
        vx = 2.0f * (q1 * q3 - q0 * q2);    // 用当前四元数推算出的重力方向（旋转矩阵第三行）
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        ex = ay * vz - az * vy;             // 两个方向的叉积 = 误差：方向是需要补偿的转轴，大小约等于偏差角
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;
        gx += T2_MAHONY_KP * ex;            // 把误差按比例加到角速度上，姿态就会慢慢转回正确方向
        gy += T2_MAHONY_KP * ey;
        gz += T2_MAHONY_KP * ez;
    }

    // 第 2 步：四元数积分  q(新) = q + 0.5 × q ⊗ [0, ω] × dt
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    att->q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz);
    att->q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy);
    att->q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx);
    att->q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx);

    // 第 3 步：归一化，再换算成欧拉角给屏幕显示
    att_normalize(att);
    att_to_euler(att);
}

void T2_Attitude_zero_yaw(T2_Attitude_Struct *att)
{
    att_from_euler(att, att->roll / RAD_TO_DEG, att->pitch / RAD_TO_DEG, 0.0f);
}

//============================================ 零偏校准 ============================================
// 原理：静止时陀螺仪的真实角速度是 0，读到的平均值就是零偏（每个芯片都不一样，还随温度变化）。
// 静置采集 2 s 求平均，之后每次读数都减去它。如果采集期间模块被碰了，平均值就不准了，
// 所以同时记录每个轴的最大、最小值，峰峰值超过 T2_CALIB_MOTION_LIMIT_DPS 就清空重来。
void T2_Calib_start(T2_Calib_Struct *c)
{
    uint8_t i;
    c->count = 0;
    for (i = 0; i < 3U; i++) {
        c->sum_g[i] = 0.0f;
        c->sum_a[i] = 0.0f;
        c->min_g[i] = 1e9f;
        c->max_g[i] = -1e9f;
    }
    c->state = T2_CALIB_RUNNING;
}

uint8_t T2_Calib_feed(T2_Calib_Struct *c, const float gyro_dps[3], const float accel_g[3])
{
    uint8_t i;
    if (c->state != T2_CALIB_RUNNING) {
        return c->state;
    }
    for (i = 0; i < 3U; i++) {
        c->sum_g[i] += gyro_dps[i];
        c->sum_a[i] += accel_g[i];
        if (gyro_dps[i] < c->min_g[i]) c->min_g[i] = gyro_dps[i];
        if (gyro_dps[i] > c->max_g[i]) c->max_g[i] = gyro_dps[i];
        if (c->max_g[i] - c->min_g[i] > T2_CALIB_MOTION_LIMIT_DPS) {
            c->restarts++;                  // 被碰到了：清空重来
            T2_Calib_start(c);
            return c->state;
        }
    }
    c->count++;
    if (c->count >= CALIB_SAMPLES) {        // 采够了：求平均
        for (i = 0; i < 3U; i++) {
            c->gyro_bias[i]  = c->sum_g[i] / (float)c->count;
            c->accel_mean[i] = c->sum_a[i] / (float)c->count;
        }
        c->state = T2_CALIB_DONE;
    }
    return c->state;
}
