#include "NEUQ_lib_include.h"
#include <math.h>

Attitude_t att;

/* 积分项限幅（rad/s），防止长时间 windup 放大漂移 */
#define ATT_INTEGRAL_LIMIT  0.05f
/* 静止时三轴零偏在线更新的平滑系数（每 10ms 一次，越小越慢，0.005≈2s 时间常数） */
#define ATT_BIAS_TAU        0.005f

void Attitude_Init(void){
    //水平姿态
    att.q0 = 1.0f;
    att.q1 = 0.0f;
    att.q2 = 0.0f;
    att.q3 = 0.0f;

    /* Kp 越大收敛越快但会抖，Ki 用来消零漂，常用经验值 */
    att.Kp = 2.0f;
    att.Ki = 0.005f;

    att.integral_x = 0.0f;
    att.integral_y = 0.0f;
    att.integral_z = 0.0f;

    att.roll  = 0.0f;
    att.pitch = 0.0f;
    att.yaw   = 0.0f;

    att.gyro_bias_x = 0.0f;
    att.gyro_bias_y = 0.0f;
    att.gyro_bias_z = 0.0f;
}




//AI
/* ============================================================
 * 函数：Attitude_CalibrateGyro
 * 作用：上电静止 1 秒内，采 200 次陀螺仪取平均，作为零偏
 *       之后每次更新时先减掉这个零偏
 * ============================================================ */
void Attitude_CalibrateGyro(void)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    const int N = 400;              /* 200→400：约 2 秒，零偏估计更稳 */

    for (int i = 0; i < N; i++){
        NEUQ_MPU6050_read_data();
        /* 注意：库返回的是 °/s，这里先转成 rad/s 再累加 */
        sum_x += MPU6050_gyro_transition(mpu6050_gyro_x) * DEG2RAD;
        sum_y += MPU6050_gyro_transition(mpu6050_gyro_y) * DEG2RAD;
        sum_z += MPU6050_gyro_transition(mpu6050_gyro_z) * DEG2RAD;
        NEUQ_Delay_ms(5);   /* 5ms 一次，200 次约 1 秒 */
    }

    att.gyro_bias_x = sum_x / N;
    att.gyro_bias_y = sum_y / N;
    att.gyro_bias_z = sum_z / N;
}

/* ============================================================
 * 函数：Attitude_Update  —— 核心！每 10ms 调一次
 * 参数：
 *   gx, gy, gz : 陀螺仪角速度，单位 **rad/s**（你要自己从 °/s 转过来）
 *   ax, ay, az : 加速度计，单位任意（后面会归一化）
 *   dt         : 距上次调用的时间，单位 秒（这里是 0.01f）
 * ============================================================ */
void Attitude_Update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt)
{
    float norm;
    float vx, vy, vz;          /* 预测的重力方向 */
    float ex, ey, ez;          /* 预测 vs 实测 的误差（叉积） */
    float q0 = att.q0, q1 = att.q1, q2 = att.q2, q3 = att.q3;
    float gx_raw = gx, gy_raw = gy, gz_raw = gz;   /* 未减零偏的原始值（在线校准用） */

    /* ① 先减掉上电校准的零偏 */
    gx -= att.gyro_bias_x;
    gy -= att.gyro_bias_y;
    gz -= att.gyro_bias_z;

    /* ② 加速度计归一化（只关心方向，不关心大小） */
    norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    float acc_mag = norm;            /* 保存原始加速度模长（单位 g），供静止判定 */
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* ③ 由当前四元数反推"重力在机体坐标系下应该指向哪" */
    vx = 2.0f * (q1*q3 - q0*q2);
    vy = 2.0f * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /* ④ 叉积算误差：预测重力 × 实测重力 */
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    /* ④.5 静止检测 + 三轴零偏在线更新：
       判定完全基于"原始"量：加速度模长≈1g（无线性加速度）且
       原始角速度模长很小（无转动）——不依赖校正后的 gx/gy，
       避免 x/y 零偏不准导致判定死锁；
       触发时用原始角速度缓慢修正三轴零偏，把静止漂移吃掉，
       转动（角速度模长超 0.30 rad/s≈17°/s）自动停止更新 */
    {
        float gmag = sqrtf(gx_raw*gx_raw + gy_raw*gy_raw + gz_raw*gz_raw);
        if (fabsf(acc_mag - 1.0f) < 0.15f && gmag < 0.30f){
            att.gyro_bias_x += ATT_BIAS_TAU * (gx_raw - att.gyro_bias_x);
            att.gyro_bias_y += ATT_BIAS_TAU * (gy_raw - att.gyro_bias_y);
            att.gyro_bias_z += ATT_BIAS_TAU * (gz_raw - att.gyro_bias_z);
        }
    }

    /* ⑤ 积分器：把误差慢慢攒起来，进一步消残余零漂（带限幅防 windup） */
    att.integral_x += att.Ki * ex * dt;
    att.integral_y += att.Ki * ey * dt;
    /* z 轴无磁力计参考，积分只会积累噪声、放大 yaw 漂移，故弃用（仅保留 P 修正） */
    if (att.integral_x >  ATT_INTEGRAL_LIMIT) att.integral_x =  ATT_INTEGRAL_LIMIT;
    if (att.integral_x < -ATT_INTEGRAL_LIMIT) att.integral_x = -ATT_INTEGRAL_LIMIT;
    if (att.integral_y >  ATT_INTEGRAL_LIMIT) att.integral_y =  ATT_INTEGRAL_LIMIT;
    if (att.integral_y < -ATT_INTEGRAL_LIMIT) att.integral_y = -ATT_INTEGRAL_LIMIT;

    /* ⑥ 把 PI 修正量加到陀螺仪角速度上 */
    gx += att.Kp * ex + att.integral_x;
    gy += att.Kp * ey + att.integral_y;
    gz += att.Kp * ez;      /* 注意：z 轴不加积分项，见⑤ */

    /* ⑦ 四元数微分方程：q̇ = 0.5 * q ⊗ ω  （离散积分） */
    float hdt = 0.5f * dt;
    q0 += (-q1*gx - q2*gy - q3*gz) * hdt;
    q1 += ( q0*gx + q2*gz - q3*gy) * hdt;
    q2 += ( q0*gy - q1*gz + q3*gx) * hdt;
    q3 += ( q0*gz + q1*gy - q2*gx) * hdt;

    /* ⑧ 归一化四元数，防止数值误差漂移 */
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    norm = 1.0f / norm;
    att.q0 = q0 * norm;
    att.q1 = q1 * norm;
    att.q2 = q2 * norm;
    att.q3 = q3 * norm;

    /* ⑨ 把四元数翻译成欧拉角（弧度），给人看 */
    /* 俯仰角 pitch */
    float sinp = 2.0f * (att.q0*att.q2 - att.q1*att.q3);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    att.pitch = asinf(sinp);

    /* 横滚角 roll */
    att.roll = atan2f(2.0f*(att.q0*att.q1 + att.q2*att.q3),
                      1.0f - 2.0f*(att.q1*att.q1 + att.q2*att.q2));

    /* 偏航角 yaw：仅由陀螺仪积分得到，无磁力计/外部航向参考修正，
       长时间运行会缓慢漂移，属原理性限制（非代码错误） */
    att.yaw  = atan2f(2.0f*(att.q0*att.q3 + att.q1*att.q2),
                      1.0f - 2.0f*(att.q2*att.q2 + att.q3*att.q3));
}
