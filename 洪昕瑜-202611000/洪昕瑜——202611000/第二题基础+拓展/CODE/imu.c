#include "imu.h"
#include <math.h>

float raw_axis[AX_NUM], filt_axis[AX_NUM];
static uint8_t filt_ready[AX_NUM];
float att_roll, att_pitch, att_yaw;
static float gyro_bias[3];

void IMU_Init(void) {
    NEUQ_MPU6050_init();
    /* 库 init 在 WHO_AM_I 检查失败时会提前 return，跳过量程配置，导致陀螺停在默认 ±250dps、
       而代码按 ±2000dps(16.4) 换算，yaw 会被放大 8 倍。这里等器件就绪后重写一次量程再校准。 */
    NEUQ_Delay_ms(100);
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_SCALE);
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, MPU6050_GYRO_CONFIG, MPU6050_GYRO_SCALE);
}

void Filter_Reset(void) { for (int i = 0; i < AX_NUM; i++) filt_ready[i] = 0; }

/* 一阶低通滤波 */
void Filter_Update(void) {
    float a = param.filter_percent / 100.0f;
    if (a > 1) a = 1; else if (a <= 0) a = 0.01f;
    for (int i = 0; i < AX_NUM; i++)
        filt_axis[i] = filt_ready[i] ? filt_axis[i] + a * (raw_axis[i] - filt_axis[i])
                                     : (filt_ready[i] = 1, raw_axis[i]);
}

/* 姿态：四元数互补滤波
   roll/pitch 由重力方向持续修正，yaw 随整体姿态一致旋转*/
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   /* 姿态四元数（初始水平） */

void Attitude_Update(void) {
    float ax = MPU6050_acc_transition(mpu6050_acc_x);
    float ay = MPU6050_acc_transition(mpu6050_acc_y);
    float az = MPU6050_acc_transition(mpu6050_acc_z);
    /* 陀螺 -> rad/s：减零偏、乘比例修正 */
    float gx = (MPU6050_gyro_transition(mpu6050_gyro_x) - gyro_bias[0]) * GYRO_SCALE_K * 0.0174533f;
    float gy = (MPU6050_gyro_transition(mpu6050_gyro_y) - gyro_bias[1]) * GYRO_SCALE_K * 0.0174533f;
    float gz = (MPU6050_gyro_transition(mpu6050_gyro_z) - gyro_bias[2]) * GYRO_SCALE_K * 0.0174533f;

    /* 归一化加速度作为重力方向 */
    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an > 0.0001f) { ax /= an; ay /= an; az /= an; }

    /* 由四元数估计的重力方向 */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 重力误差叉积 -> 修正 roll/pitch（yaw 无基准，不反馈） */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float accel_err = fabsf(an - 1.0f);
    float kp = MAHONY_KP;
    if (accel_err > 0.25f) kp = 0.0f;
    else if (accel_err > 0.10f) kp = MAHONY_KP * (0.25f - accel_err) / 0.15f;
    gx += kp * ex;
    gy += kp * ey;

    /* 四元数一阶积分 */
    float ht = 0.5f * SAMPLE_DT;
    float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * ht;
    float dq1 = ( q0 * gx + q2 * gz - q3 * gy) * ht;
    float dq2 = ( q0 * gy - q1 * gz + q3 * gx) * ht;
    float dq3 = ( q0 * gz + q1 * gy - q2 * gx) * ht;
    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

    /* 归一化四元数 */
    float qn = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qn > 1e-6f) { q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn; }

    /* 四元数 -> 欧拉角（度）；yaw 由 atan2 天然落在 [-180,180] */
    att_roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
    att_pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.29578f;
    att_yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
}

/* 将当前 yaw 清零、保留 roll/pitch：用当前欧拉角重建四元数 */
void Zero_Yaw(void) {
    float hr = att_roll  * 0.00872665f;   /* 0.5 * deg -> rad */
    float hp = att_pitch * 0.00872665f;
    float cr = cosf(hr), sr = sinf(hr);
    float cp = cosf(hp), sp = sinf(hp);
    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;
    att_yaw = 0.0f;
}

/* 零偏在线跟踪（可追踪温漂） */
void Bias_Track(void) {
    static uint8_t still_cnt[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {                 /* AX_GX..AX_GZ 连续 */
        float e = raw_axis[AX_GX + i] - gyro_bias[i];
        if (fabsf(e) < STILL_DPS) {
            if (still_cnt[i] < 255) still_cnt[i]++;
            if (still_cnt[i] >= 40)               /* 连续 ~200ms 静止才更新零偏 */
                gyro_bias[i] += BIAS_K * e;
        } else {
            still_cnt[i] = 0;
        }
    }
}

/* 上电静止校准：取约 1 秒陀螺零偏均值，消除 yaw 静止漂移（期间保持板子静止） */
void Gyro_Calibrate(void) {
    float sum[3] = {0, 0, 0};
    for (int i = 0; i < 200; i++) {
        NEUQ_MPU6050_read_data();
        sum[0] += MPU6050_gyro_transition(mpu6050_gyro_x);
        sum[1] += MPU6050_gyro_transition(mpu6050_gyro_y);
        sum[2] += MPU6050_gyro_transition(mpu6050_gyro_z);
        NEUQ_Delay_ms(5);
    }
    gyro_bias[0] = sum[0] / 200.0f;
    gyro_bias[1] = sum[1] / 200.0f;
    gyro_bias[2] = sum[2] / 200.0f;
}
