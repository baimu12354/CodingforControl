/*
 * NEUQ_Attitude.c
 * 四元数姿态解算
 *
 * 为什么不用欧拉角直接积分：
 *   欧拉角微分方程里有 1/cos(pitch) 这一项，俯仰角到±90度就除零了（万向节死锁），
 *   而且三次三角函数算得慢、误差累积也快。
 *   四元数只有4个数，处处没有奇异点，只要一个微分方程 q' = 0.5*q⊗ω，
 *   最后再转成欧拉角显示就行。
 *
 * Mahony互补滤波在干什么：
 *   陀螺积分短期准、长期因为零偏会漂；加速度计长期准（一直指着重力）、短期噪声大。
 *   把加速度计测到的重力方向，和四元数推算出来的重力方向做叉乘，得到姿态误差e，
 *   再把e按比例Kp、按积分Ki反馈到角速度上一起去积分，姿态就被重力慢慢拉回来了。
 *       ω' = ω + Kp*e + Ki*∫e dt
 *   这里没有磁力计，所以只能修正roll和pitch。yaw没有任何观测量能修正它，
 *   只能靠陀螺积分，零偏剩多少就一直积多少，所以它一定会慢慢漂，这是六轴的固有性质。
 *
 * 归一化：
 *   浮点积分会让|q|慢慢偏离1，每次更新完要除以模长，
 *   不然相当于姿态里混进一个缩放，欧拉角越算越错。
 */
#include "NEUQ_lib_include.h"
#include <math.h>
#include "NEUQ_IMU.h"
#include "NEUQ_Attitude.h"

#define NEUQ_RAD2DEG        (57.29577951f)
#define NEUQ_DEG2RAD        (0.0174532925f)

#define NEUQ_DT_MIN_S       (0.0005f)       // 0.5ms，dt太小则不采信，防止数值爆炸
#define NEUQ_DT_MAX_S       (0.0500f)       // 50ms，切页卡顿后防止一次积分太多

NEUQ_Attitude_t g_att;

// Mahony的积分项，单位rad/s，只在Ki>0时用
static float s_integral[3];

// 把角度规整到(-180,180]
static float NEUQ_Att_Wrap180(float deg)
{
    while(deg >  180.0f) deg -= 360.0f;
    while(deg <= -180.0f) deg += 360.0f;
    return deg;
}

void NEUQ_Attitude_Init(void)
{
    g_att.q[0] = 1.0f;      // 单位四元数，相当于假设模块水平放着
    g_att.q[1] = 0.0f;
    g_att.q[2] = 0.0f;
    g_att.q[3] = 0.0f;
    g_att.roll = g_att.pitch = g_att.yaw = g_att.yaw_raw = 0.0f;
    g_att.yaw_zero = 0.0f;
    g_att.dt_ms = (float)NEUQ_IMU_PERIOD_MS;
    s_integral[0] = s_integral[1] = s_integral[2] = 0.0f;
}

// 用一次加速度计采样把四元数直接摆到当前姿态，不用等它慢慢收敛
void NEUQ_Attitude_InitFromAccel(const float acc_g[3])
{
    float ax = acc_g[0];
    float ay = acc_g[1];
    float az = acc_g[2];
    float n;
    float roll, pitch;
    float cr, sr, cp, sp;

    NEUQ_Attitude_Init();       // 先全部复位

    n = sqrtf(ax * ax + ay * ay + az * az);

    // 模长明显不是1g，说明模块在动或者没接好，那就还是按水平处理
    if(!(n >= NEUQ_IMU_ACC_TRUST_MIN && n <= NEUQ_IMU_ACC_TRUST_MAX))
    {
        return;
    }

    ax /= n; ay /= n; az /= n;

    // 静止时重力在机体坐标下就是(0,0,1)，反推roll和pitch
    roll  = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    cr = cosf(roll  * 0.5f);  sr = sinf(roll  * 0.5f);
    cp = cosf(pitch * 0.5f);  sp = sinf(pitch * 0.5f);

    // ZYX顺序，yaw取0
    g_att.q[0] =  cp * cr;
    g_att.q[1] =  cp * sr;
    g_att.q[2] =  sp * cr;
    g_att.q[3] = -sp * sr;

    g_att.roll    = roll  * NEUQ_RAD2DEG;
    g_att.pitch   = pitch * NEUQ_RAD2DEG;
    g_att.yaw     = 0.0f;
    g_att.yaw_raw = 0.0f;
}

void NEUQ_Attitude_SetGains(float kp, float ki)
{
    g_att.kp = kp;
    g_att.ki = ki;
}

void NEUQ_Attitude_SetYawZero(void)
{
    g_att.yaw_zero = g_att.yaw_raw;
    g_att.yaw      = 0.0f;
}

// 一次完整的姿态更新：扣零偏 -> 加速度计修正 -> 四元数积分 -> 归一化 -> 转欧拉角
void NEUQ_Attitude_Update(const float acc_g[3], const float gyro_dps[3], float dt_s)
{
    float q0 = g_att.q[0], q1 = g_att.q[1], q2 = g_att.q[2], q3 = g_att.q[3];
    float gx, gy, gz;               // rad/s，准备拿去积分的角速度
    float ax, ay, az;
    float norm;
    float twoKp = 2.0f * g_att.kp;
    float twoKi = 2.0f * g_att.ki;
    float halfT;
    float qa, qb, qc;
    uint8_t accel_ok;

    // dt保护，!(dt_s >= MIN)这样写顺便把NaN也挡住了
    if(!(dt_s >= NEUQ_DT_MIN_S)) dt_s = (float)NEUQ_IMU_PERIOD_MS / 1000.0f;
    if(dt_s >  NEUQ_DT_MAX_S)    dt_s = NEUQ_DT_MAX_S;
    g_att.dt_ms = dt_s * 1000.0f;

    // 加速度计模长。接近1g才认为它测的是重力，被推被敲、快速平移的时候不采信
    ax = acc_g[0];
    ay = acc_g[1];
    az = acc_g[2];
    norm = sqrtf(ax * ax + ay * ay + az * az);
    accel_ok = (uint8_t)((norm >= NEUQ_IMU_ACC_TRUST_MIN) &&
                         (norm <= NEUQ_IMU_ACC_TRUST_MAX));

    // 减掉开机静态校准出来的零偏，剩下的残余就照实积分。
    // yaw没有观测量能修正它，所以这点残余会积成慢慢漂的航向，这是六轴的固有性质。
    gx = (gyro_dps[0] - g_imu.gyro_bias_dps[0]) * NEUQ_DEG2RAD;
    gy = (gyro_dps[1] - g_imu.gyro_bias_dps[1]) * NEUQ_DEG2RAD;
    gz = (gyro_dps[2] - g_imu.gyro_bias_dps[2]) * NEUQ_DEG2RAD;

    // 加速度计修正，只修roll和pitch                                                 
    if(accel_ok)
    {
        float halfvx, halfvy, halfvz;   // 四元数推算出的重力方向
        float halfex, halfey, halfez;   // 和实测重力方向的误差

        ax /= norm; ay /= norm; az /= norm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 叉乘得到误差，方向就是该往哪边修正
        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;

        if(twoKi > 0.0f)
        {
            s_integral[0] += twoKi * halfex * dt_s;
            s_integral[1] += twoKi * halfey * dt_s;
            s_integral[2] += twoKi * halfez * dt_s;
            gx += s_integral[0];
            gy += s_integral[1];
            gz += s_integral[2];
        }
        else
        {
            s_integral[0] = s_integral[1] = s_integral[2] = 0.0f;
        }

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 四元数微分方程积分：q += 0.5*q⊗ω*dt
    halfT = 0.5f * dt_s;
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * halfT;
    q1 += ( qa * gx + qc * gz - q3 * gy) * halfT;
    q2 += ( qa * gy - qb * gz + q3 * gx) * halfT;
    q3 += ( qa * gz + qb * gy - qc * gx) * halfT;

    // 归一化
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if(norm < 0.000001f)
    {
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    }
    else
    {
        norm = 1.0f / norm;
        q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
    }

    g_att.q[0] = q0; g_att.q[1] = q1; g_att.q[2] = q2; g_att.q[3] = q3;

    // 四元数转欧拉角，Z-Y-X顺序
    g_att.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2)) * NEUQ_RAD2DEG;

    {
        float sp = 2.0f * (q0 * q2 - q3 * q1);
        if(sp >  1.0f) sp =  1.0f;      // asinf参数必须钳位，不然出NaN
        if(sp < -1.0f) sp = -1.0f;
        g_att.pitch = asinf(sp) * NEUQ_RAD2DEG;
    }

    g_att.yaw_raw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                           1.0f - 2.0f * (q2 * q2 + q3 * q3)) * NEUQ_RAD2DEG;

    // yaw以校准那一刻为相对零点显示
    g_att.yaw = NEUQ_Att_Wrap180(g_att.yaw_raw - g_att.yaw_zero);
}
