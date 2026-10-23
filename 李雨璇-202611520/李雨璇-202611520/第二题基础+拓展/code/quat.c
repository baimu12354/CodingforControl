#include "NEUQ_lib_include.h"
#include <math.h>

extern float ax;
extern float ay;
extern float az;
extern float gx;
extern float gy;
extern float gz;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

float pitch;
float roll;
float yaw;

// Mahony 参数
float Kp = 2.0f;       // PI 比例增益
float Ki = 0.01f;      // PI 积分增益
float halfT = 0.005f;  // 采样周期的一半

#define DEG_TO_RAD  0.01745329252f   // pai / 180
#define RAD_TO_DEG  57.2957795f      // 180 / pai

// IMU Update Mahony 姿态解算
void IMU_Update(void){
    float vx, vy, vz;
    float ex, ey, ez;
    float norm;
    float nax, nay, naz;
    float gxc, gyc, gzc;

    // 积分误差
    static float exInt = 0, eyInt = 0, ezInt = 0;

    // 数据限幅
    float accMag = sqrtf(ax*ax + ay*ay + az*az);
    if (accMag < 0.01f || accMag > 4.0f)
        return;
    if (gx > 500.0f || gx < -500.0f ||
        gy > 500.0f || gy < -500.0f ||
        gz > 500.0f || gz < -500.0f)
        return;

    // 加速度计归一化
    norm = 1.0f / accMag;
    nax = ax * norm;
    nay = ay * norm;
    naz = az * norm;

    // 由当前四元数估计的重力方向（机体坐标系）
    vx = 2.0f * (q1*q3 - q0*q2);
    vy = 2.0f * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 叉乘求误差：测量重力与估计重力
    ex = nay*vz - naz*vy;
    ey = naz*vx - nax*vz;
    ez = nax*vy - nay*vx;

    // 误差积分
    if (Ki > 0.0f){
        exInt += ex * Ki;
        eyInt += ey * Ki;
        ezInt += ez * Ki;
    } else {
        exInt = 0; eyInt = 0; ezInt = 0;
    }

    // 陀螺仪角速度：deg/s 转 rad/s，再 PI 补偿
    gxc = gx * DEG_TO_RAD + Kp*ex + exInt;
    gyc = gy * DEG_TO_RAD + Kp*ey + eyInt;
    gzc = gz * DEG_TO_RAD / 180 + Kp*ez + ezInt;

    // 四元数微分方程更新（一阶）
    q0 += (-q1*gxc - q2*gyc - q3*gzc) * halfT;
    q1 += ( q0*gxc + q2*gzc - q3*gyc) * halfT;
    q2 += ( q0*gyc - q1*gzc + q3*gxc) * halfT;
    q3 += ( q0*gzc + q1*gyc - q2*gxc) * halfT;

    // 四元数归一化，防止数值漂移发散
    norm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    // 四元数转欧拉角
    roll  = asinf(2.0f * (q1*q3 - q0*q2)) * RAD_TO_DEG;
    pitch = -atan2f(2.0f * q2*q3 + 2.0f * q0*q1, q0*q0 + q1*q1 - q2*q2 - q3*q3) * RAD_TO_DEG;
    yaw   = atan2f(2.0f*(q1*q2 + q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3) * RAD_TO_DEG;
}