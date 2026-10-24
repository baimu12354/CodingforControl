#include "NEUQ_lib_include.h"
#include <math.h>

FILTER_PARAMS g_filter_param;
float g_raw_angle = 0.0f;
float g_filt_angle = 0.0f;
float filtered_acc[3] = {0};
float filtered_gyro[3] = {0};

#ifndef FILTER_DT
#define FILTER_DT 0.02f
#endif
#define RAD      (57.29577951308232f)
#define DEG2RAD (0.0174532925199433f)

/* 四元数 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float euler_roll = 0.0f, euler_pitch = 0.0f, euler_yaw = 0.0f;

/* Mahony 增益 */
static float cfg_kp = 1.0f;
static float cfg_ki = 0.0f;
static float i_fbx = 0.0f, i_fby = 0.0f, i_fbz = 0.0f;

/* ===== 陀  螺零偏（°/s）===== */
static float gyro_off[3] = {0};
static uint8_t calib_ok = 0;

static uint8_t calib_run = 0;
static uint16_t calib_cnt = 0;
static float calib_sum[3] = {0};
#define CALIB_SAMPLES 200

/* ===== 加速度计零偏（g）===== */
static float acc_off[3] = {0};
static uint8_t acc_calib_ok = 0;

static uint8_t acc_calib_run = 0;
static uint16_t acc_calib_cnt = 0;
static float acc_calib_sum[3] = {0};
#define ACC_CALIB_SAMPLES 200

/* ===== 水平归零偏移（°）===== */
static float roll_offset = 0.0f;
static float pitch_offset = 0.0f;
static uint8_t level_zeroed = 0;

/* ===== 合并校准状态 ===== */
static uint8_t combined_calib_busy = 0;
static uint8_t combined_calib_done = 0;

/* ===== 对外状态查询 ===== */
uint8_t sensor_calibrate_busy(void) { return combined_calib_busy; }
uint8_t sensor_calibrated(void)      { return combined_calib_done; }

static void quat_normalize(void) {
    float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (n > 1e-6f) {
        n = 1.0f / n;
        q0 *= n; q1 *= n; q2 *= n; q3 *= n;
    } else {
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    }
}

static void quat_to_euler(void) {
    float sinr = 2.0f*(q0*q1 + q2*q3);
    float cosr = 1.0f - 2.0f*(q1*q1 + q2*q2);
    euler_roll = atan2f(sinr, cosr) * RAD;

    float sinp = 2.0f*(q0*q2 - q3*q1);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    euler_pitch = asinf(sinp) * RAD;

    float siny = 2.0f*(q0*q3 + q1*q2);
    float cosy = 1.0f - 2.0f*(q2*q2 + q3*q3);
    euler_yaw = atan2f(siny, cosy) * RAD;
}

static void mahony_update(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    if ((ax==0.0f) && (ay==0.0f) && (az==0.0f)) return;

    norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    ax /= norm; ay /= norm; az /= norm;

    float halfvx = q1*q3 - q0*q2;
    float halfvy = q0*q1 + q2*q3;
    float halfvz = q0*q0 - 0.5f + q3*q3;

    float ex = ay*halfvz - az*halfvy;
    float ey = az*halfvx - ax*halfvz;
    float ez = ax*halfvy - ay*halfvx;

    if (cfg_ki > 0.0f) {
        i_fbx += cfg_ki * ex * FILTER_DT;
        i_fby += cfg_ki * ey * FILTER_DT;
        i_fbz += cfg_ki * ez * FILTER_DT;
        gx += i_fbx; gy += i_fby; gz += i_fbz;
    } else {
        i_fbx = 0.0f; i_fby = 0.0f; i_fbz = 0.0f;
    }

    gx += cfg_kp * ex;
    gy += cfg_kp * ey;
    gz += cfg_kp * ez;

    float hx = gx * 0.5f * FILTER_DT;
    float hy = gy * 0.5f * FILTER_DT;
    float hz = gz * 0.5f * FILTER_DT;

    float q0t = q0, q1t = q1, q2t = q2;
    q0 += (-q1t*hx - q2t*hy - q3*hz);
    q1 += ( q0t*hx + q2t*hz - q3*hy);
    q2 += ( q0t*hy - q1t*hz + q3*hx);
    q3 += ( q0t*hz + q1t*hy - q2t*hx);

    quat_normalize();
}

/* ===== 陀螺校准喂数（由PIT2调用，原有接口保留）===== */
void gyro_calibrate_feed(void) {
    if (!calib_run) return;
    calib_sum[0] += MPU6050_gyro_transition(mpu6050_gyro_x);
    calib_sum[1] += MPU6050_gyro_transition(mpu6050_gyro_y);
    calib_sum[2] += MPU6050_gyro_transition(mpu6050_gyro_z);
    calib_cnt++;
    if (calib_cnt >= CALIB_SAMPLES) {
        gyro_off[0] = calib_sum[0] / (float)calib_cnt;
        gyro_off[1] = calib_sum[1] / (float)calib_cnt;
        gyro_off[2] = calib_sum[2] / (float)calib_cnt;
        calib_ok = 1;
        calib_run = 0;
    }
}

/* ===== 加速度计校准喂数 ===== */
void acc_calibrate_feed(void) {
    if (!acc_calib_run) return;
    acc_calib_sum[0] += MPU6050_acc_transition(mpu6050_acc_x);
    acc_calib_sum[1] += MPU6050_acc_transition(mpu6050_acc_y);
    acc_calib_sum[2] += MPU6050_acc_transition(mpu6050_acc_z);
    acc_calib_cnt++;
    if (acc_calib_cnt >= ACC_CALIB_SAMPLES) {
        acc_off[0] = acc_calib_sum[0] / (float)acc_calib_cnt - 0.0f;
        acc_off[1] = acc_calib_sum[1] / (float)acc_calib_cnt - 0.0f;
        acc_off[2] = acc_calib_sum[2] / (float)acc_calib_cnt - 1.0f;
        acc_calib_ok = 1;
        acc_calib_run = 0;
    }
}

/* ===== 合并校准：同时启动陀螺+加速度计校准 ===== */
void sensor_calibrate_start(void) {
    calib_run = 1;
    calib_cnt = 0;
    calib_sum[0] = calib_sum[1] = calib_sum[2] = 0.0f;
    calib_ok = 0;

    acc_calib_run = 1;
    acc_calib_cnt = 0;
    acc_calib_sum[0] = acc_calib_sum[1] = acc_calib_sum[2] = 0.0f;
    acc_calib_ok = 0;

    combined_calib_busy = 1;
    combined_calib_done = 0;

    level_zeroed = 0;
    roll_offset = 0.0f;
    pitch_offset = 0.0f;
}

/* ===== 检查两个校准是否完成并自动归零 ===== */
static void combined_calib_check(void) {
    if (!combined_calib_busy) return;
    if (calib_ok && acc_calib_ok && !combined_calib_done) {
        static uint8_t settle_cnt = 0;
        settle_cnt++;
        if (settle_cnt >= 10) {
            roll_offset = euler_roll;
            pitch_offset = euler_pitch;
            level_zeroed = 1;
            combined_calib_done = 1;
            combined_calib_busy = 0;
            settle_cnt = 0;
        }
    }
}

void filter_init(void) {
    g_filter_param.alpha = 0.98f;
    g_filter_param.gyro_threshold = 1.0f;
    g_filter_param.acc_threshold = 0.0f;
    g_filter_param.selected_axis = FILTER_AXIS_X;
    g_filter_param.send_enable = 0;
    g_filter_param.checksum = 0;

    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    euler_roll = euler_pitch = euler_yaw = 0.0f;
    i_fbx = i_fby = i_fbz = 0.0f;

    gyro_off[0] = gyro_off[1] = gyro_off[2] = 0.0f;
    calib_ok = 0;
    calib_run = 0;
    calib_cnt = 0;

    acc_off[0] = acc_off[1] = acc_off[2] = 0.0f;
    acc_calib_ok = 0;
    acc_calib_run = 0;
    acc_calib_cnt = 0;

    roll_offset = pitch_offset = 0.0f;
    level_zeroed = 0;
    combined_calib_busy = 0;
    combined_calib_done = 0;
}

void filter_updata(void) {
    /* 校准喂数 */
    gyro_calibrate_feed();
    acc_calibrate_feed();
    combined_calib_check();

    float acc[3] = {
        MPU6050_acc_transition(mpu6050_acc_x),
        MPU6050_acc_transition(mpu6050_acc_y),
        MPU6050_acc_transition(mpu6050_acc_z)
    };
    float gyr[3] = {
        MPU6050_gyro_transition(mpu6050_gyro_x),
        MPU6050_gyro_transition(mpu6050_gyro_y),
        MPU6050_gyro_transition(mpu6050_gyro_z)
    };

    /* 减加速度计零偏 */
    if (acc_calib_ok) {
        acc[0] -= acc_off[0];
        acc[1] -= acc_off[1];
        acc[2] -= acc_off[2];
    }

    cfg_kp = g_filter_param.gyro_threshold;
    cfg_ki = g_filter_param.acc_threshold;

    /* 一阶低通 */
    for (int i = 0; i < 3; i++) {
        filtered_acc[i]  = filtered_acc[i]*(1.0f - g_filter_param.alpha) + acc[i]*g_filter_param.alpha;
        filtered_gyro[i] = filtered_gyro[i]*(1.0f - g_filter_param.alpha) + gyr[i]*g_filter_param.alpha;
    }

    /* 单轴加速度角 */
    float acc_angle = 0.0f;
    switch (g_filter_param.selected_axis) {
        case FILTER_AXIS_X: acc_angle = atan2f(acc[1], acc[2]) * RAD; break;
        case FILTER_AXIS_Y: acc_angle = atan2f(acc[0], acc[2]) * RAD; break;
        case FILTER_AXIS_Z: acc_angle = atan2f(acc[0], acc[1]) * RAD; break;
        default: break;
    }
    g_raw_angle = acc_angle;

    /* 陀螺减零偏，°/s -> rad/s */
    float gxr = (gyr[0] - gyro_off[0]) * DEG2RAD;
    float gyr2= (gyr[1] - gyro_off[1]) * DEG2RAD;
    float gzr = (gyr[2] - gyro_off[2]) * DEG2RAD;

    mahony_update(gxr, gyr2, gzr, acc[0], acc[1], acc[2]);
    quat_to_euler();

    switch (g_filter_param.selected_axis) {
        case FILTER_AXIS_X: g_filt_angle = euler_roll;  break;
        case FILTER_AXIS_Y: g_filt_angle = euler_pitch; break;
        case FILTER_AXIS_Z: g_filt_angle = euler_yaw;   break;
        default: g_filt_angle = euler_roll; break;
    }
}

float filter_get_roll(void)  {
    if (level_zeroed) return euler_roll - roll_offset;
    return euler_roll;
}
float filter_get_pitch(void) {
    if (level_zeroed) return euler_pitch - pitch_offset;
    return euler_pitch;
}
float filter_get_yaw(void)   { return euler_yaw; }
float filter_get_raw(void)   { return g_raw_angle; }
float filter_get_filter(void){ return g_filt_angle; }
void  filter_set_axis(uint8_t axis) { if (axis <= FILTER_AXIS_Z) g_filter_param.selected_axis = axis; }
uint8_t filter_get_axis(void) { return g_filter_param.selected_axis; }

/* ===== 对外保留的陀螺校准接口（兼容旧调用）===== */
void gyro_calibrate_start(void) {
    sensor_calibrate_start();   /* 统一入口，一次同时校陀螺+加速度计 */
}
uint8_t gyro_calibrate_busy(void) { return combined_calib_busy; }
uint8_t gyro_calibrated(void)     { return calib_ok; }
float gyro_get_offset(uint8_t axis) {
    if (axis <= 2) return gyro_off[axis];
    return 0.0f;
}