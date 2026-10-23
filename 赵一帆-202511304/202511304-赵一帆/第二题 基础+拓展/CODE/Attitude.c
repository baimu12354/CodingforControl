#include "Attitude.h"
#include "IMU.h"    
#include <math.h>

EulerAngle_t euler = {0};
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float exInt = 0, eyInt = 0, ezInt = 0;
static float bias_x = 0, bias_y = 0, bias_z = 0;
static float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;  // g / deg/s

#define KP 8.0f
#define KI 0.01f
#define DEG_TO_RAD 0.0174533f
#define RAD_TO_DEG 57.29578f
#define HALF_DT 0.0025f   // dt=5ms 的一半

void Attitude_Init(void) {
    q0=1; q1=q2=q3=0; exInt=eyInt=ezInt=0;
    euler.roll = euler.pitch = euler.yaw = 0;
    // 零偏校准（上电静止约1秒）
    float sx=0, sy=0, sz=0;
    for (int i=0; i<500; i++) {
        NEUQ_MPU6050_read_data();
        sx += MPU6050_gyro_transition(mpu6050_gyro_x);
        sy += MPU6050_gyro_transition(mpu6050_gyro_y);
        sz += MPU6050_gyro_transition(mpu6050_gyro_z);
        NEUQ_Delay_ms(2);
    }
    bias_x = sx/500; bias_y = sy/500; bias_z = sz/500;
}

void Attitude_Update(void) {

    ax = imu.ax; ay = imu.ay; az = imu.az;
    gx = imu.gx; gy = imu.gy; gz = imu.gz;

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    ax/=norm; ay/=norm; az/=norm;

    float vx = 2*(q1*q3 - q0*q2);
    float vy = 2*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    exInt += ex*KI; eyInt += ey*KI; ezInt += ez*KI;

    float rx = (gx - bias_x)*DEG_TO_RAD + KP*ex + exInt;
    float ry = (gy - bias_y)*DEG_TO_RAD + KP*ey + eyInt;
    float rz = (gz - bias_z)*DEG_TO_RAD + KP*ez + ezInt;

    float q0_=q0, q1_=q1, q2_=q2, q3_=q3;
    q0 += (-q1_*rx - q2_*ry - q3_*rz)*HALF_DT;
    q1 += ( q0_*rx + q2_*rz - q3_*ry)*HALF_DT;
    q2 += ( q0_*ry - q1_*rz + q3_*rx)*HALF_DT;
    q3 += ( q0_*rz + q1_*ry - q2_*rx)*HALF_DT;

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0/=norm; q1/=norm; q2/=norm; q3/=norm;

   // 计算出中间项
    float sin_p = 2.0f * (q0 * q2 - q3 * q1);
    float cos_p = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    
    // 限幅
    if (sin_p > 1.0f)  sin_p = 1.0f;
    if (sin_p < -1.0f) sin_p = -1.0f;

    euler.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

    euler.pitch = asinf(sin_p) * RAD_TO_DEG; 
    
    euler.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}