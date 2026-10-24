/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* 文件名称          NEUQ_Interrupts.c
* 修改记录
* 2026-10-11        考生        考核二：数据管线重构（架构级改进）：
*                              1) PIT中断只做“读MPU6050原始数据 -> 压环形缓冲”一件事，立即退出；
*                                 六轴滤波、零偏标定、Mahony解算全部移到主循环 IMU_Pipeline_RunOnce()；
*                              2) 六轴与姿态数据用结构体打包，主循环经 IMU_GetSnapshot() 取快照；
*                              3) 开机零偏标定为阻塞式（PIT启动前连续采200点，2s完成）。
********************************************************************************************************************/
#include "../code/NEUQ_lib/NEUQ_lib_include.h"
#include <math.h>
#include <string.h>
//================================ 数据管线参数 ================================//
#define IMU_SAMPLE_DT              0.01f      // PIT采样周期，单位 s（10ms）
#define IMU_DEG_TO_RAD             0.017453293f
#define IMU_RAD_TO_DEG             57.2957795f
#define IMU_ALPHA_MIN              0.010f     // 六轴低通系数允许范围
#define IMU_ALPHA_MAX              1.000f
#define IMU_ALPHA_DEFAULT          0.150f     // 开机默认系数
#define IMU_ATT_ACC_ALPHA          0.10f      // 姿态解算专用加速度预滤波系数
#define MAHONY_TWO_KP              2.00f      // Mahony 比例增益
#define MAHONY_TWO_KI              0.03f      // Mahony 积分增益
//================================ 环形缓冲 ================================//
static uint8_t      s_fifo_mem[IMU_FIFO_SAMPLES * sizeof(IMU_RawSample_t)];
static NEUQ_FIFO_t  s_fifo;
//================================ 管线内部状态 ================================//
static uint8_t  s_mpu_ok = 0;
static float    s_alpha = IMU_ALPHA_DEFAULT;
static uint8_t  s_upload_enable = 0;
static uint8_t  s_upload_axis = 0;
static float    s_flt[IMU_AXIS_NUM];           // 六轴低通历史状态
static uint8_t  s_flt_inited = 0;
static float    s_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // 四元数
static float    s_integ_fb[3] = {0.0f, 0.0f, 0.0f}; // Mahony积分反馈
static float    s_att_acc[3] = {0.0f, 0.0f, 1.0f};  // 姿态用预滤波加速度
static uint8_t  s_calib_state = IMU_CALIB_IDLE;     // 标定状态
static float    s_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static uint32_t s_sample_count = 0;
static IMU_Snapshot_t s_snap;                 // 对外发布的快照
/* ======================= 外部引用 ======================= */
extern NEUQ_FIFO_t g_rx_fifo;                 // 串口接收FIFO，定义在 NEUQ_Main.c 中
//================================ 基础数学工具 ================================//
static float IMU_InvSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    int32_t i;
    if (x <= 0.0f) return 0.0f;
    memcpy(&i, &y, 4);
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, 4);
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
static float IMU_Clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
// 用加速度初始化四元数
static void IMU_Quat_SetFromAccel(float ax, float ay, float az)
{
    float norm = sqrtf(ax * ax + ay * ay + az);
    float roll, pitch, cr, sr, cp, sp;
    if (norm < 1e-4f) {
        s_q[0] = 1.0f; s_q[1] = 0.0f; s_q[2] = 0.0f; s_q[3] = 0.0f;
        return;
    }
    ax /= norm; ay /= norm; az /= norm;
    roll  = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    cr = cosf(roll * 0.5f);  sr = sinf(roll * 0.5f);
    cp = cosf(pitch * 0.5f); sp = sinf(pitch * 0.5f);
    s_q[0] = cr * cp; s_q[1] = sr * cp; s_q[2] = cr * sp; s_q[3] = -sr * sp;
}
// Mahony互补滤波
static void IMU_Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az, float two_kp, float two_ki, float dt)
{
    float recip_norm, halfvx, halfvy, halfvz, halfex, halfey, halfez, qa, qb, qc;
    float q0 = s_q[0], q1 = s_q[1], q2 = s_q[2], q3 = s_q[3];
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recip_norm = IMU_InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm; ay *= recip_norm; az *= recip_norm;
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
        if (two_ki > 0.0f) {
            s_integ_fb[0] += two_ki * halfex * dt;
            s_integ_fb[1] += two_ki * halfey * dt;
            s_integ_fb[2] += two_ki * halfez * dt;

            // 积分抗饱和限幅，防止长时间静置漂移
            s_integ_fb[0] = IMU_Clampf(s_integ_fb[0], -1.0f, 1.0f);
            s_integ_fb[1] = IMU_Clampf(s_integ_fb[1], -1.0f, 1.0f);
            s_integ_fb[2] = IMU_Clampf(s_integ_fb[2], -1.0f, 1.0f);
            gx += s_integ_fb[0]; gy += s_integ_fb[1]; gz += s_integ_fb[2];
        }
        gx += two_kp * halfex;
        gy += two_kp * halfey;
        gz += two_kp * halfez;
    }
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);
    recip_norm = IMU_InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    s_q[0] = q0 * recip_norm; s_q[1] = q1 * recip_norm;
    s_q[2] = q2 * recip_norm; s_q[3] = q3 * recip_norm;
}
// 四元数转欧拉角
static void IMU_Quat_ToEuler(float *roll, float *pitch, float *yaw)
{
    float q0 = s_q[0], q1 = s_q[1], q2 = s_q[2], q3 = s_q[3];
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp >= 1.0f) sinp = 1.0f;
    if (sinp <= -1.0f) sinp = -1.0f;
    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * IMU_RAD_TO_DEG;
    *pitch = asinf(sinp) * IMU_RAD_TO_DEG;
    *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * IMU_RAD_TO_DEG;
}
//================================ 零偏标定 ================================//
void IMU_Calib_Blocking(void)
{
    float gsum[3] = {0.0f, 0.0f, 0.0f};
    float asum[3] = {0.0f, 0.0f, 0.0f};
    float aavg[3];
    uint32_t n;
    uint8_t i;
    s_calib_state = IMU_CALIB_IDLE;
    if (!s_mpu_ok) return;
    for (n = 0; n < IMU_CALIB_SAMPLES; n++) {
        NEUQ_MPU6050_read_data();
        gsum[0] += MPU6050_gyro_transition(mpu6050_gyro_x);
        gsum[1] += MPU6050_gyro_transition(mpu6050_gyro_y);
        gsum[2] += MPU6050_gyro_transition(mpu6050_gyro_z);
        asum[0] += MPU6050_acc_transition(mpu6050_acc_x);
        asum[1] += MPU6050_acc_transition(mpu6050_acc_y);
        asum[2] += MPU6050_acc_transition(mpu6050_acc_z);
        HAL_Delay(10);
    }
    for (i = 0; i < 3; i++) {
        s_gyro_bias[i] = gsum[i] / (float)IMU_CALIB_SAMPLES;
        aavg[i]        = asum[i] / (float)IMU_CALIB_SAMPLES;
    }
    s_att_acc[0] = aavg[0]; s_att_acc[1] = aavg[1]; s_att_acc[2] = aavg[2];
    IMU_Quat_SetFromAccel(aavg[0], aavg[1], aavg[2]);
    s_integ_fb[0] = s_integ_fb[1] = s_integ_fb[2] = 0.0f;
    s_calib_state = IMU_CALIB_DONE;
}
//================================ 对外接口 ================================//
void IMU_Pipeline_Init(uint8_t mpu_ok)
{
    uint8_t i;
    NEUQ_FIFO_init(&s_fifo, s_fifo_mem, sizeof(s_fifo_mem));
    s_sample_count = 0;
    s_mpu_ok = mpu_ok;
    s_alpha = IMU_ALPHA_DEFAULT;
    s_upload_enable = 0;
    s_upload_axis = 0;
    s_flt_inited = 0;
    for (i = 0; i < IMU_AXIS_NUM; i++) s_flt[i] = 0.0f;
    s_q[0] = 1.0f; s_q[1] = 0.0f; s_q[2] = 0.0f; s_q[3] = 0.0f;
    s_integ_fb[0] = s_integ_fb[1] = s_integ_fb[2] = 0.0f;
    s_att_acc[0] = 0.0f; s_att_acc[1] = 0.0f; s_att_acc[2] = 1.0f;
    s_gyro_bias[0] = s_gyro_bias[1] = s_gyro_bias[2] = 0.0f;
    s_calib_state = IMU_CALIB_IDLE;
    memset(&s_snap, 0, sizeof(s_snap));
    s_snap.filter_alpha = IMU_ALPHA_DEFAULT;
    s_snap.quat[0] = 1.0f;
    s_snap.mpu_ok = mpu_ok;
}
void IMU_Set_Alpha(float alpha)
{
    s_alpha = IMU_Clampf(alpha, IMU_ALPHA_MIN, IMU_ALPHA_MAX);
    s_snap.filter_alpha = s_alpha;
}
void IMU_Set_Upload(uint8_t enable)
{
    s_upload_enable = enable ? 1u : 0u;
    s_snap.upload_enable = s_upload_enable;
}
void IMU_Set_UploadAxis(uint8_t axis)
{
    if (axis >= IMU_AXIS_NUM) axis = IMU_AXIS_NUM - 1u;
    s_upload_axis = axis;
    s_snap.upload_axis = axis;
}
void IMU_GetSnapshot(IMU_Snapshot_t *dst)
{
    if (dst == 0) return;
    *dst = s_snap;
}
//================================ 主循环管线：取一帧 -> 滤波 -> Mahony解算 ================================//
uint8_t IMU_Pipeline_RunOnce(void)
{
    IMU_RawSample_t raw;
    float acc_g[3], gyro_dps[3];
    float a, b, roll, pitch, yaw, bg[3];
    uint8_t i;
    if (NEUQ_FIFO_read_all(&s_fifo, (uint8_t *)&raw, sizeof(raw)) != NEUQ_FIFO_OK) return 0;
    acc_g[0] = MPU6050_acc_transition(raw.acc_adc[0]);
    acc_g[1] = MPU6050_acc_transition(raw.acc_adc[1]);
    acc_g[2] = MPU6050_acc_transition(raw.acc_adc[2]);
    gyro_dps[0] = MPU6050_gyro_transition(raw.gyro_adc[0]);
    gyro_dps[1] = MPU6050_gyro_transition(raw.gyro_adc[1]);
    gyro_dps[2] = MPU6050_gyro_transition(raw.gyro_adc[2]);
    a = IMU_Clampf(s_alpha, IMU_ALPHA_MIN, IMU_ALPHA_MAX);
    b = 1.0f - a;
    if (!s_flt_inited) {
        for (i = 0; i < 3; i++) { s_flt[i] = acc_g[i]; s_flt[i + 3] = gyro_dps[i]; }
        s_flt_inited = 1;
    } else {
        for (i = 0; i < 3; i++) {
            s_flt[i]     = a * acc_g[i]     + b * s_flt[i];
            s_flt[i + 3] = a * gyro_dps[i]  + b * s_flt[i + 3];
        }
    }
    s_sample_count++;
    if (s_calib_state == IMU_CALIB_DONE) {
        for (i = 0; i < 3; i++) {
            s_att_acc[i] = IMU_ATT_ACC_ALPHA * acc_g[i] + (1.0f - IMU_ATT_ACC_ALPHA) * s_att_acc[i];
            bg[i] = (gyro_dps[i] - s_gyro_bias[i]) * IMU_DEG_TO_RAD;
        }
        IMU_Mahony_Update(bg[0], bg[1], bg[2], s_att_acc[0], s_att_acc[1], s_att_acc[2], MAHONY_TWO_KP, MAHONY_TWO_KI, IMU_SAMPLE_DT);
    }
    IMU_Quat_ToEuler(&roll, &pitch, &yaw);
    for (i = 0; i < 3; i++) {
        s_snap.acc_g[i] = acc_g[i]; s_snap.gyro_dps[i] = gyro_dps[i];
        s_snap.acc_adc[i] = raw.acc_adc[i]; s_snap.gyro_adc[i] = raw.gyro_adc[i];
        s_snap.acc_g_flt[i] = s_flt[i]; s_snap.gyro_dps_flt[i] = s_flt[i + 3];
        s_snap.gyro_bias[i] = s_gyro_bias[i];
    }
    s_snap.temperature = raw.temperature;
    s_snap.quat[0] = s_q[0]; s_snap.quat[1] = s_q[1];
    s_snap.quat[2] = s_q[2]; s_snap.quat[3] = s_q[3];
    s_snap.roll_deg = roll; s_snap.pitch_deg = pitch; s_snap.yaw_deg = yaw;
    s_snap.sample_count = s_sample_count; s_snap.mpu_ok = s_mpu_ok;
    s_snap.filter_alpha = s_alpha; s_snap.upload_enable = s_upload_enable; s_snap.upload_axis = s_upload_axis;
    return 1;
}
//================================ 中断服务函数 ================================//
// PIT采样中断（10ms）：只做读取与入队，绝不在这里做滤波解算
void NEUQ_PIT_1_CALLBACK(void)
{
    IMU_RawSample_t sample;
    NEUQ_MPU6050_read_data();
    sample.acc_adc[0] = mpu6050_acc_x; sample.acc_adc[1] = mpu6050_acc_y; sample.acc_adc[2] = mpu6050_acc_z;
    sample.gyro_adc[0] = mpu6050_gyro_x; sample.gyro_adc[1] = mpu6050_gyro_y; sample.gyro_adc[2] = mpu6050_gyro_z;
    sample.temperature = MPU6050_Data_get.Temperature;
    NEUQ_FIFO_write_all(&s_fifo, (uint8_t *)&sample, sizeof(sample));
}
void NEUQ_PIT_2_CALLBACK(void) {}
// 串口接收中断：把字节塞进 g_rx_fifo
void TS_UART_0_RX_CALLBACK(uint8_t dat) { NEUQ_FIFO_write_byte(&g_rx_fifo, dat); }
void TS_UART_1_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_2_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_3_RX_CALLBACK(uint8_t dat) { (void)dat; }
// 外部中断（空实现，防链接报错）
void NEUQ_EXTI0_Callback(void)  {}
void NEUQ_EXTI1_Callback(void)  {}
void NEUQ_EXTI2_Callback(void)  {}
void NEUQ_EXTI3_Callback(void)  {}
void NEUQ_EXTI4_Callback(void)  {}
void NEUQ_EXTI5_Callback(void)  {}
void NEUQ_EXTI6_Callback(void)  {}
void NEUQ_EXTI7_Callback(void)  {}
void NEUQ_EXTI8_Callback(void)  {}
void NEUQ_EXTI9_Callback(void)  {}
void NEUQ_EXTI10_Callback(void) {}
void NEUQ_EXTI11_Callback(void) {}
void NEUQ_EXTI12_Callback(void) {}
void NEUQ_EXTI13_Callback(void) {}
void NEUQ_EXTI14_Callback(void) {}
void NEUQ_EXTI15_Callback(void) {}
