/*********************************************************************************************************************
* 文件名称          NEUQ_Attitude
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第二题（挑战 20 分）：四元数姿态解算 实现
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include <math.h>
#include "NEUQ_IMU.h"           /* 需要 MPU6050 的原始量与换算函数 */
#include "NEUQ_Attitude.h"

volatile float att_roll  = 0.0f;
volatile float att_pitch = 0.0f;
volatile float att_yaw   = 0.0f;

static float s_q0 = 1.0f, s_q1 = 0.0f, s_q2 = 0.0f, s_q3 = 0.0f;

#define DT      0.005f          /* 5 ms，和 IMU_PERIOD_MS 一致 */
#define KP      2.0f            /* 互补滤波比例系数 */
#define D2R     0.0174533f
#define R2D     57.29578f

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     用当前重力方向构造四元数，并把三轴角置零（"此刻就是 0°"）
* 备注信息     水平放置时 az≈+1：横滚=俯仰=0，四元数就是单位四元数。
*              若加速度模长明显不是 1 g，说明传感器还没读到有效数据（或模块在自由落体），
*              这时直接返回、不改任何状态，避免用垃圾数据把姿态搞乱。
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Attitude_Reset(void)
{
    float ax = MPU6050_acc_transition(mpu6050_acc_x);
    float ay = MPU6050_acc_transition(mpu6050_acc_y);
    float az = MPU6050_acc_transition(mpu6050_acc_z);
    float n  = sqrtf(ax * ax + ay * ay + az * az);
    float roll, pitch, cr, sr, cp, sp;

    if (n < 0.5f || n > 1.5f)       /* 静止时模长应该接近 1 g */
    {
        return;
    }

    ax /= n;
    ay /= n;
    az /= n;

    roll  = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    cr = cosf(roll  * 0.5f);  sr = sinf(roll  * 0.5f);
    cp = cosf(pitch * 0.5f);  sp = sinf(pitch * 0.5f);

    /* q = qz(0) ⊗ qy(pitch) ⊗ qx(roll)，航向取 0 */
    s_q0 =  cp * cr;
    s_q1 =  cp * sr;
    s_q2 =  sp * cr;
    s_q3 = -sp * sr;

    /* 数据有效才把三轴角置零 */
    att_roll  = 0.0f;
    att_pitch = 0.0f;
    att_yaw   = 0.0f;
}

void NEUQ_Attitude_Init(void)
{
    NEUQ_Attitude_Reset();
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     四元数姿态更新（5 ms 一次）
* 输入参数     gx/gy/gz : 已扣零偏的角速度 dps
*              ax/ay/az : 加速度 g
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Attitude_Update(float gx, float gy, float gz,
                          float ax, float ay, float az)
{
    float q0 = s_q0, q1 = s_q1, q2 = s_q2, q3 = s_q3;
    float rx = gx * D2R, ry = gy * D2R, rz = gz * D2R;   /* rad/s */
    float n;

    n = sqrtf(ax * ax + ay * ay + az * az);
    if (n > 0.001f)
    {
        float vx, vy, vz, ex, ey, ez;

        ax /= n; ay /= n; az /= n;

        /* ② 四元数估计的重力方向 */
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* ③ 误差 e = a × v，乘 Kp 修正角速度 */
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        rx += KP * ex;
        ry += KP * ey;
        rz += KP * ez;
    }

    /* ④ 四元数微分 */
    s_q0 = q0 + 0.5f * (-q1 * rx - q2 * ry - q3 * rz) * DT;
    s_q1 = q1 + 0.5f * ( q0 * rx + q2 * rz - q3 * ry) * DT;
    s_q2 = q2 + 0.5f * ( q0 * ry - q1 * rz + q3 * rx) * DT;
    s_q3 = q3 + 0.5f * ( q0 * rz + q1 * ry - q2 * rx) * DT;

    /* ⑤ 归一化 */
    n = sqrtf(s_q0 * s_q0 + s_q1 * s_q1 + s_q2 * s_q2 + s_q3 * s_q3);
    if (n > 0.0001f)
    {
        s_q0 /= n; s_q1 /= n; s_q2 /= n; s_q3 /= n;
    }

    /* ⑥ 欧拉角（航向带倾斜补偿，俯仰接近 ±90° 时不会乱跳） */
    q0 = s_q0; q1 = s_q1; q2 = s_q2; q3 = s_q3;

    att_roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                       1.0f - 2.0f * (q1 * q1 + q2 * q2)) * R2D;

    n = 2.0f * (q0 * q2 - q3 * q1);
    if (n >  1.0f) { n =  1.0f; }
    if (n < -1.0f) { n = -1.0f; }
    att_pitch = asinf(n) * R2D;

    att_yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                       1.0f - 2.0f * (q2 * q2 + q3 * q3)) * R2D;
}
