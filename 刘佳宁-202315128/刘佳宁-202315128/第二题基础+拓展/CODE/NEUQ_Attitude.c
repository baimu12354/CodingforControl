/*********************************************************************************************************************
* 四元数姿态解算（Mahony 互补滤波）
*
* 原理（写给新手看，尽量大白话）：
*   1. 陀螺仪积分能快速跟上旋转，但会慢慢“漂移”。
*   2. 加速度计静止时能测出重力方向，能用来纠正横滚/俯仰的漂移，
*      但它不能分辨水平原地转（所以航向角只能靠陀螺仪积分，会缓慢漂移，这是正常的）。
*   3. Mahony 滤波的思路：把“实际测到的重力方向”和“四元数算出来的重力方向”
*      做叉乘得到误差，再用误差去纠正陀螺仪，从而保持长期不漂。
*
* 姿态用「四元数」表示：q = q0 + q1*i + q2*j + q3*k
*   q0 是实部，q1/q2/q3 是虚部。四元数归一化后即可表示任意旋转。
********************************************************************************************************************/
#include "NEUQ_Attitude.h"
#include "NEUQ_lib_include.h"
#include <math.h>

// 三个姿态角输出（单位：度）
float att_roll  = 0.0f;
float att_pitch = 0.0f;
float att_yaw   = 0.0f;

// 当前姿态四元数，初始为“单位四元数”(1,0,0,0)，表示没有旋转
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// 陀螺仪零偏：静置时陀螺仪也会输出一个小角速度，之后每次都要减掉它
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// 上一次解算的时刻（单位：微秒），用来算两次之间的时间差 dt
static uint16_t last_us = 0;

// 角度 <-> 弧度 换算
#define DEG2RAD (0.01745329252f)
#define RAD2DEG (57.295779513f)

// Mahony 比例系数：越大，加速度纠正越快（一般取 0.5 ~ 2 之间）
#define KP (2.0f)

// 静置校准采样次数（每次间隔 2ms，约 2 秒完成）
#define CALI_SAMPLES (1000)

//----------------------------------------------------------------------------
// 静置校准：采样陀螺仪求零偏，并把四元数、姿态角复位
// 要求：调用时模块保持完全静止（放平、别动），约 2 秒完成。
// 上电时由 NEUQ_Attitude_Init 调用一次；之后也可以在菜单里长按按键重新调用。
//----------------------------------------------------------------------------
void NEUQ_Attitude_Calibrate(void)
{
    int i;
    float sum[3] = {0.0f, 0.0f, 0.0f};

    // 连续采样，累加三轴角速度，再取平均得到零偏
    for(i = 0; i < CALI_SAMPLES; i++){
        NEUQ_MPU6050_read_data();
        sum[0] += MPU6050_gyro_transition(mpu6050_gyro_x);
        sum[1] += MPU6050_gyro_transition(mpu6050_gyro_y);
        sum[2] += MPU6050_gyro_transition(mpu6050_gyro_z);
        NEUQ_Delay_ms(2);
    }
    gyro_bias[0] = sum[0] / CALI_SAMPLES;
    gyro_bias[1] = sum[1] / CALI_SAMPLES;
    gyro_bias[2] = sum[2] / CALI_SAMPLES;

    // 四元数回到单位四元数，姿态角清零（Yaw 以当前朝向作为 0 度参考）
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    att_roll = 0.0f; att_pitch = 0.0f; att_yaw = 0.0f;

    // 记录当前时刻（微秒），供解算时计算 dt
    last_us = (uint16_t)NEUQ_Get_Timer_Value_us();
}

//----------------------------------------------------------------------------
// 上电初始化：先让陀螺仪上电稳定一会儿，再做静置校准
//----------------------------------------------------------------------------
void NEUQ_Attitude_Init(void)
{
    // 微秒计时器（TIM13）已在 User_Main 里通过 NEUQ_Timer_init() 启动，这里直接使用
    NEUQ_Delay_ms(500);        // 等陀螺仪上电稳定，再开始校准
    NEUQ_Attitude_Calibrate();
}

//----------------------------------------------------------------------------
// 四元数姿态解算（每采到一组数据调用一次）
//----------------------------------------------------------------------------
void NEUQ_Attitude_Solve(float acc_g[3], float gyro_deg[3])
{
    float dt, gx, gy, gz;
    float ax, ay, az, norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 1) 计算本次与上次之间的时间差 dt（秒）
    uint16_t now_us = (uint16_t)NEUQ_Get_Timer_Value_us();
    uint16_t dt_us  = (uint16_t)(now_us - last_us);   // 无符号相减，自动处理定时器回绕
    last_us = now_us;
    dt = (float)dt_us / 1000000.0f;                   // 微秒 -> 秒
    if(dt <= 0.0f || dt > 0.1f) dt = 0.01f;           // 防止出现异常时间

    // 2) 角速度减零偏，并换算成 弧度/秒
    gx = (gyro_deg[0] - gyro_bias[0]) * DEG2RAD;
    gy = (gyro_deg[1] - gyro_bias[1]) * DEG2RAD;
    gz = (gyro_deg[2] - gyro_bias[2]) * DEG2RAD;

    // 3) 加速度归一化成单位向量（只看方向，不看大小）
    ax = acc_g[0]; ay = acc_g[1]; az = acc_g[2];
    norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm < 1e-6f) norm = 1.0f;            // 防止除以 0
    ax /= norm; ay /= norm; az /= norm;

    // 4) 由当前四元数算出“理论上的重力方向”
    vx = 2.0f * (q1*q3 - q0*q2);
    vy = 2.0f * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 5) 误差 = 实际加速度 叉乘 理论重力，用来纠正漂移
    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

    // 6) 把误差（比例项）反馈回角速度
    gx += KP * ex;
    gy += KP * ey;
    gz += KP * ez;

    // 7) 四元数一阶积分（四元数微分方程）
    q0 += (-q1*gx - q2*gy - q3*gz) * 0.5f * dt;
    q1 += ( q0*gx + q2*gz - q3*gy) * 0.5f * dt;
    q2 += ( q0*gy - q1*gz + q3*gx) * 0.5f * dt;
    q3 += ( q0*gz + q1*gy - q2*gx) * 0.5f * dt;

    // 8) 四元数归一化（必须做，否则长时间会越算越偏）
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

    // 9) 四元数 -> 欧拉角（单位：度）
    att_roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;
    att_pitch = asinf(2.0f*(q0*q2 - q3*q1)) * RAD2DEG;
    att_yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;
}
