/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_Main
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include <math.h>
#include "Imu_State.h"

Attitude g_attitude = {1.0f, 0.0f, 0.0f, 0.0f, {0}, {0}, 0, 0, 0};

static float App_InvSqrt(float value)
{
    if (value <= 0.0f) return 0.0f;
    return 1.0f / sqrtf(value);
}

static float App_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void App_QuaternionFromRollPitch(float roll, float pitch)
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    /* 初始航向定义为 0，重力只用于建立横滚和俯仰。 */
    g_attitude.q0 = cr * cp;
    g_attitude.q1 = sr * cp;
    g_attitude.q2 = cr * sp;
    g_attitude.q3 = -sr * sp;
}

/* 静止放置约 1 s，求三轴陀螺零偏并用平均重力建立初始姿态。 */

void Attitude_Solver_Calibrate(void)
{
    uint16_t i;
    float gyro_sum[3] = {0, 0, 0};
    float acc_sum[3] = {0, 0, 0};
    float ax, ay, az;
    float roll0, pitch0;

    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(10, 30, "Keep MPU6050 still");
    NEUQ_LCD200_DisplayString(10, 70, "Calibrating gyro...");

    for (i = 0; i < APP_CALIBRATION_SAMPLES; i++)
    {
        NEUQ_MPU6050_read_data();
        gyro_sum[0] += MPU6050_gyro_transition(mpu6050_gyro_x);
        gyro_sum[1] += MPU6050_gyro_transition(mpu6050_gyro_y);
        gyro_sum[2] += MPU6050_gyro_transition(mpu6050_gyro_z);
        acc_sum[0] += MPU6050_acc_transition(mpu6050_acc_x);
        acc_sum[1] += MPU6050_acc_transition(mpu6050_acc_y);
        acc_sum[2] += MPU6050_acc_transition(mpu6050_acc_z);
        NEUQ_Delay_ms(2);
    }

    for (i = 0; i < 3U; i++)
        g_attitude.gyro_bias[i] = gyro_sum[i] / (float)APP_CALIBRATION_SAMPLES;

    ax = acc_sum[0] / (float)APP_CALIBRATION_SAMPLES;
    ay = acc_sum[1] / (float)APP_CALIBRATION_SAMPLES;
    az = acc_sum[2] / (float)APP_CALIBRATION_SAMPLES;
    roll0 = atan2f(ay, az);
    pitch0 = atan2f(-ax, sqrtf(ay * ay + az * az));
    App_QuaternionFromRollPitch(roll0, pitch0);
}

/*
 * Mahony IMU 四元数更新：陀螺负责快速转动，归一化加速度给重力方向反馈。
 * 航向没有磁力计绝对参考，只保证相对转动趋势并依靠零偏校准减小漂移。
 */

void Attitude_Solver_Update(float dt)
{
    const float kp = 1.8f;
    const float ki = 0.03f;
    float ax = g_imu.raw[0], ay = g_imu.raw[1], az = g_imu.raw[2];
    float gx = (g_imu.raw[3] - g_attitude.gyro_bias[0]) * APP_DEG_TO_RAD;
    float gy = (g_imu.raw[4] - g_attitude.gyro_bias[1]) * APP_DEG_TO_RAD;
    float gz = (g_imu.raw[5] - g_attitude.gyro_bias[2]) * APP_DEG_TO_RAD;
    float norm, vx, vy, vz, ex, ey, ez;
    float q0, q1, q2, q3;
    float dq0, dq1, dq2, dq3;
    float sin_pitch;

    norm = App_InvSqrt(ax * ax + ay * ay + az * az);
    if (norm > 0.0f)
    {
        ax *= norm; ay *= norm; az *= norm;
        q0 = g_attitude.q0; q1 = g_attitude.q1;
        q2 = g_attitude.q2; q3 = g_attitude.q3;

        /* 当前四元数预测的单位重力方向。 */
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        g_attitude.integral[0] += ki * ex * dt;
        g_attitude.integral[1] += ki * ey * dt;
        g_attitude.integral[2] += ki * ez * dt;
        gx += kp * ex + g_attitude.integral[0];
        gy += kp * ey + g_attitude.integral[1];
        gz += kp * ez + g_attitude.integral[2];
    }

    q0 = g_attitude.q0; q1 = g_attitude.q1;
    q2 = g_attitude.q2; q3 = g_attitude.q3;
    dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
    q0 += dq0 * dt; q1 += dq1 * dt; q2 += dq2 * dt; q3 += dq3 * dt;

    norm = App_InvSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 0.0f)
    {
        g_attitude.q0 = q0 * norm;
        g_attitude.q1 = q1 * norm;
        g_attitude.q2 = q2 * norm;
        g_attitude.q3 = q3 * norm;
    }

    q0 = g_attitude.q0; q1 = g_attitude.q1;
    q2 = g_attitude.q2; q3 = g_attitude.q3;
    g_attitude.roll = atan2f(2.0f * (q0*q1 + q2*q3),
                             1.0f - 2.0f * (q1*q1 + q2*q2)) * APP_RAD_TO_DEG;
    sin_pitch = App_Clamp(2.0f * (q0*q2 - q3*q1), -1.0f, 1.0f);
    g_attitude.pitch = asinf(sin_pitch) * APP_RAD_TO_DEG;
    g_attitude.yaw = atan2f(2.0f * (q0*q3 + q1*q2),
                            1.0f - 2.0f * (q2*q2 + q3*q3)) * APP_RAD_TO_DEG;
}


