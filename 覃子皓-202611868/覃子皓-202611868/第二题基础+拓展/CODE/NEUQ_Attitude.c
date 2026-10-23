#include "NEUQ_lib_include.h"
#include <math.h>

//=============================================================================================
// NEUQ_Attitude.c
// Madgwick 六轴 IMU 姿态解算
//=============================================================================================

#define RAD2DEG  57.29578f      // 180/π，弧度转角度

// ---- 模块状态 ----
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // 四元数
static float NEUQ_beta = 0.1f;                               // 算法增益，决定加速度修正力度
static float NEUQ_invSampleFreq = 1.0f / 512.0f;             // 采样周期(秒)

// 欧拉角，单位弧度（Get 系列会转成度返回）
volatile float roll  = 0.0f;
volatile float pitch = 0.0f;
volatile float yaw   = 0.0f;

static char NEUQ_anglesComputed = 0;  //标志位

// 陀螺仪零偏，单位 °/s
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

// ---- 内部函数 ----
static float NEUQ_invSqrt(float x)
{
    return 1.0f / sqrtf(x);  // 1/根号下x
}

static void NEUQ_computeAngles(void)   //四元数反解欧拉角
{
    float sp = -2.0f * (q1 * q3 - q0 * q2);
    if (sp >  1.0f) sp =  1.0f;  //限位
    if (sp < -1.0f) sp = -1.0f;

    roll  = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);  //反正切
    pitch = asinf(sp);
    yaw   = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
    NEUQ_anglesComputed = 1;  //标志位置一
}

//=============================================================================================
// 姿态解算初始化
//=============================================================================================
void NEUQ_Attitude_Init(float sampleFrequency)
{
    NEUQ_invSampleFreq = 1.0f / sampleFrequency;
    NEUQ_beta = 0.1f;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    NEUQ_anglesComputed = 0;

    roll  = 0.0f;
    pitch = 0.0f;
    yaw   = 0.0f;
}

//=============================================================================================
// 设置算法增益
//=============================================================================================
void NEUQ_Attitude_SetBeta(float beta)
{
    NEUQ_beta = beta;
}  //可设置

//=============================================================================================
// 复位姿态
//=============================================================================================
void NEUQ_Attitude_Reset(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    NEUQ_anglesComputed = 0;

    roll  = 0.0f;
    pitch = 0.0f;
    yaw   = 0.0f;
}

//=============================================================================================
// 姿态解算更新（六轴 IMU，无磁力计）
// 备注：内部逻辑与 Madgwick_UpdateIMU 完全一致
//=============================================================================================
void NEUQ_Attitude_Update(void)
{
    // ---- 读传感器，做单位换算 ----
    // 陀螺仪：°/s -> rad/s，减零偏
    float gx = (MPU6050_gyro_transition(mpu6050_gyro_x) - gyro_bias_x) * 0.017453292f;  //角度制转弧度制
    float gy = (MPU6050_gyro_transition(mpu6050_gyro_y) - gyro_bias_y) * 0.017453292f;
    float gz = (MPU6050_gyro_transition(mpu6050_gyro_z) - gyro_bias_z) * 0.017453292f;

    // 加速度计：单位 g
    float ax = MPU6050_acc_transition(mpu6050_acc_x);
    float ay = MPU6050_acc_transition(mpu6050_acc_y);
    float az = MPU6050_acc_transition(mpu6050_acc_z);

    // ---- 以下算法逻辑与 Madgwick_UpdateIMU 完全一致 ----
    float recipNorm;  //中间变量
    float s0, s1, s2, s3;  //目标函数的梯度
    float qDot1, qDot2, qDot3, qDot4;  //四元数导数
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 1) 由陀螺仪积分算四元数变化率
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // 2) 加速度计读数有效时，用"重力方向"做梯度下降修正
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {  
        recipNorm = NEUQ_invSqrt(ax * ax + ay * ay + az * az);  //加速度向量归一化变单位向量
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;  //预计算

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q2 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = NEUQ_invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;  //梯度归一化

        qDot1 -= NEUQ_beta * s0;  //修正项叠加到陀螺仪积分项
        qDot2 -= NEUQ_beta * s1;
        qDot3 -= NEUQ_beta * s2;
        qDot4 -= NEUQ_beta * s3;
    }

    // 3) 积分
    q0 += qDot1 * NEUQ_invSampleFreq;
    q1 += qDot2 * NEUQ_invSampleFreq;
    q2 += qDot3 * NEUQ_invSampleFreq;
    q3 += qDot4 * NEUQ_invSampleFreq;

    // 4) 四元数归一化
    recipNorm = NEUQ_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    NEUQ_anglesComputed = 0;
}

//=============================================================================================
// 陀螺仪零偏校准（上电静止时调用一次）
//=============================================================================================
void NEUQ_Gyro_Calibrate(uint16_t N)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;

    for (uint16_t i = 0; i < N; i++)
    {
        NEUQ_MPU6050_read_data();
        sx += MPU6050_gyro_transition(mpu6050_gyro_x);
        sy += MPU6050_gyro_transition(mpu6050_gyro_y);
        sz += MPU6050_gyro_transition(mpu6050_gyro_z);
        NEUQ_Delay_ms(2);
    }

    gyro_bias_x = sx / (float)N;
    gyro_bias_y = sy / (float)N;
    gyro_bias_z = sz / (float)N;
}

//=============================================================================================
// 姿态角（度）
//=============================================================================================
float NEUQ_Attitude_GetRoll(void)  { if (!NEUQ_anglesComputed) NEUQ_computeAngles(); return roll  * RAD2DEG; }
float NEUQ_Attitude_GetPitch(void) { if (!NEUQ_anglesComputed) NEUQ_computeAngles(); return pitch * RAD2DEG; }
float NEUQ_Attitude_GetYaw(void)   { if (!NEUQ_anglesComputed) NEUQ_computeAngles(); return yaw   * RAD2DEG + 180.0f; }

//=============================================================================================
// 姿态角（弧度）
//=============================================================================================
float NEUQ_Attitude_GetRollRadians(void)  { if (!NEUQ_anglesComputed) NEUQ_computeAngles(); return roll; }
float NEUQ_Attitude_GetPitchRadians(void) { if (!NEUQ_anglesComputed) NEUQ_computeAngles(); return pitch; }
float NEUQ_Attitude_GetYawRadians(void)   { if (!NEUQ_anglesComputed) NEUQ_computeAngles(); return yaw; }