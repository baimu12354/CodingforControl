/********************************************************************************
 * 文件    : Attitude.c
 * 功能    : 四元数姿态解算（Mahony 互补滤波）
 * 说明    : 1. 用 MPU6050 的角速度做四元数递推更新，用加速度做姿态修正（互补）
 *           2. 每步四元数均做归一化，保证数值长期稳定
 *           3. 陀螺仪零偏由 Task_Gyro.c 在静置校准时测出，通过
 *              Attitude_SetGyroBias() 传入，这里负责扣除
 *           4. 六轴传感器无磁力计，航向只能由角速度积分得到（相对航向），
 *              会随时间缓慢漂移，属正常现象
 ********************************************************************************/
#include "NEUQ_lib_include.h"
#include <math.h>
#include "Attitude.h"

/*=========================== 滤波参数 ===========================*/
// Mahony 比例增益：越大加速度计修正越强、收敛越快，但也越容易引入振动噪声。
// 本工程 100Hz、模块手持/桌面静置场景下 0.5 较合适。
#define KP  0.5f
// Mahony 积分增益：这里依赖上电/按键静态零偏校准，不启用在线积分项，避免航向被积分误差带偏。
#define KI  0.0f

#define DEG2RAD (3.14159265358979f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265358979f)

/*=========================== 内部状态 ===========================*/
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // 四元数 [q0,q1,q2,q3]=[w,x,y,z]
static float gb_x = 0.0f, gb_y = 0.0f, gb_z = 0.0f;       // 陀螺仪零偏 (deg/s)
static float ierr_x = 0.0f, ierr_y = 0.0f, ierr_z = 0.0f;  // 积分误差（KI=0 时不用）

float att_roll = 0.0f, att_pitch = 0.0f, att_yaw = 0.0f;   // 姿态角（度）

/*=========================== 工具函数 ===========================*/

static float inv_sqrt(float x){
    return 1.0f / sqrtf(x);   // STM32F407 带单精度 FPU，够快
}

/*=========================== 对外接口 ===========================*/

void Attitude_Init(void){
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    gb_x = gb_y = gb_z = 0.0f;
    ierr_x = ierr_y = ierr_z = 0.0f;
    att_roll = att_pitch = att_yaw = 0.0f;
}

void Attitude_SetGyroBias(float bx, float by, float bz){
    gb_x = bx; gb_y = by; gb_z = bz;
}

void Attitude_SetInitFromAccel(float ax, float ay, float az){
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if(n < 1e-4f) return;               // 加速度无效，保持现状
    ax /= n; ay /= n; az /= n;

    // 由重力方向求初始横滚/俯仰（与"六轴显示"页的几何关系一致）
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    // 由 roll/pitch 构造初始四元数（航向 = 0）
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    q0 = cp * cr;
    q1 = cp * sr;
    q2 = sp * cr;
    q3 = -sp * sr;

    // 归一化
    n = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= n; q1 *= n; q2 *= n; q3 *= n;

    att_yaw = 0.0f;                     // 航向以当前方向为相对参考
}

void Attitude_Update(float gx, float gy, float gz,
                     float ax, float ay, float az, float dt){
    float recip;
    float halfT = 0.5f * dt;

    // 1) 扣除零偏并换算到 rad/s
    float wx = (gx - gb_x) * DEG2RAD;
    float wy = (gy - gb_y) * DEG2RAD;
    float wz = (gz - gb_z) * DEG2RAD;

    // 2) 加速度计校正（互补滤波的"修正"通道）
    float norm = ax * ax + ay * ay + az * az;
    if(norm > 0.0f){                    // 加速度有效时才做修正
        recip = inv_sqrt(norm);
        ax *= recip; ay *= recip; az *= recip;

        // 估计的重力方向在机体系中的投影："上"方向 = R^T * [0,0,1]
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

        // 误差 = 测量值 × 估计值（叉积）
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        // 比例反馈
        wx += KP * ex;
        wy += KP * ey;
        wz += KP * ez;

        // 积分反馈（KI=0 时为空转，保留以便需要时启用）
        if(KI > 0.0f){
            ierr_x += KI * ex * dt;
            ierr_y += KI * ey * dt;
            ierr_z += KI * ez * dt;
            wx += ierr_x;
            wy += ierr_y;
            wz += ierr_z;
        }
    }

    // 3) 四元数递推：q += 0.5 * q ⊗ [0, wx, wy, wz] * dt
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * wx - qc * wy - q3 * wz) * halfT;
    q1 += ( qa * wx + qc * wz - q3 * wy) * halfT;
    q2 += ( qa * wy - qb * wz + q3 * wx) * halfT;
    q3 += ( qa * wz + qb * wy - qc * wx) * halfT;

    // 4) 四元数归一化
    recip = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recip; q1 *= recip; q2 *= recip; q3 *= recip;

    // 5) 四元数转欧拉角（Z-Y-X，航空常用，单位：度）
    float r11 = 1.0f - 2.0f * (q2*q2 + q3*q3);
    float r21 = 2.0f * (q1*q2 + q0*q3);
    float r31 = 2.0f * (q1*q3 - q0*q2);
    float r32 = 2.0f * (q2*q3 + q0*q1);
    float r33 = 1.0f - 2.0f * (q1*q1 + q2*q2);

    att_roll  = atan2f(r32, r33) * RAD2DEG;
    att_pitch = asinf(-r31) * RAD2DEG;
    att_yaw   = atan2f(r21, r11) * RAD2DEG;
}
