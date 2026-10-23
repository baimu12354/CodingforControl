#include "NEUQ_lib_include.h"
#include "attitude.h"
//============================================姿态解算===========================================================
Attitude_t g_attitude;

#define GYRO_LSB_PER_DPS    16.4f        //量程系数
#define COMP_ALPHA          0.96f        //互补滤波系数
#define DT                  0.01f        //s
#define RAD_TO_DEG          57.2957795f  //弧度转角度系数
static float s_bias_x = 0.0f;
static float s_bias_y = 0.0f;
static float s_bias_z = 0.0f; 
//==========================静止======================
void Attitude_Ini(void){
    g_attitude.pitch = 0.0f;
    g_attitude.roll  = 0.0f;
    g_attitude.yaw   = 0.0f;
    int32_t sum_x=0,sum_y=0,sum_z=0;
    const uint16_t N=200;
//            m/s^2
    for (uint16_t i=0;i<N;i++){
        NEUQ_MPU6050_read_data();
        sum_x+=mpu6050_gyro_x;
        sum_y+=mpu6050_gyro_y;
        sum_z+=mpu6050_gyro_z;
        NEUQ_Delay_ms(20);//采样间隔
    }
    s_bias_x=(float)sum_x/(float)N;
    s_bias_y=(float)sum_y/(float)N;
    s_bias_z=(float)sum_z/(float)N;
    NEUQ_MPU6050_read_data();
    float ax = (float)mpu6050_acc_x;
    float ay = (float)mpu6050_acc_y;
    float az = (float)mpu6050_acc_z;
    g_attitude.pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    g_attitude.roll  = atan2f(ay, az) * RAD_TO_DEG;
    g_attitude.yaw=0.0f;
    float pitch_rad=g_attitude.pitch/RAD_TO_DEG;
    float roll_rad=g_attitude.roll/RAD_TO_DEG;
    float yaw_rad=g_attitude.yaw/RAD_TO_DEG;

    float cy=cosf(yaw_rad*0.5f);
    float sy=sinf(yaw_rad*0.5f);
    float cp=cosf(pitch_rad*0.5f);
    float sp=sinf(pitch_rad*0.5f);
    float cr=cosf(roll_rad*0.5f);
    float sr=sinf(roll_rad*0.5f);

    g_attitude.q0=cy*cp*cr+sy*sp*sr;
    g_attitude.q1=cy*cp*sr-sy*sp*cr;
    g_attitude.q2=sy*cp*sr+cy*sp*cr;
    g_attitude.q3=sy*cp*cr-cy*sp*sr;
}

//=========================中断============================
void Attitude_Update(void){
    float ax=(float)mpu6050_acc_x;
    float ay=(float)mpu6050_acc_y;
    float az=(float)mpu6050_acc_z;
    //============减去零偏============真实角速度
    float gx = ((float)mpu6050_gyro_x - s_bias_x) / GYRO_LSB_PER_DPS*0.0174533f;
    float gy = ((float)mpu6050_gyro_y - s_bias_y) / GYRO_LSB_PER_DPS*0.0174533f;
    float gz = ((float)mpu6050_gyro_z - s_bias_z) / GYRO_LSB_PER_DPS*0.0174533f;

    // 2. 加速度计归一化 (把加速度变成单位向量)
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.001f) return; // 防止除以0
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // 3. Mahony 算法核心 (用加速度计修正陀螺仪漂移)
    // 估计重力方向
    float vx = 2.0f * (g_attitude.q1 * g_attitude.q3 - g_attitude.q0 * g_attitude.q2);
    float vy = 2.0f * (g_attitude.q0 * g_attitude.q1 + g_attitude.q2 * g_attitude.q3);
    float vz = g_attitude.q0 * g_attitude.q0 - g_attitude.q1 * g_attitude.q1 - g_attitude.q2 * g_attitude.q2 + g_attitude.q3 * g_attitude.q3;
    
    // 误差计算 (向量外积)
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);
    
    // PI 修正陀螺仪 ( Ki 和 Kp)
    // Ki 用于消除静态误差，Kp 用于加速收敛
    static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
    float Ki = 0.001f; 
    float Kp = 1.5f;   
    
    integralFBx += Ki * ex * DT;
    integralFBy += Ki * ey * DT;
    integralFBz += Ki * ez * DT;
    
    gx += Kp * ex + integralFBx;
    gy += Kp * ey + integralFBy;
    gz += Kp * ez + integralFBz;
    
    //四元数更新 (一阶龙格库塔法)
    float q0 = g_attitude.q0, q1 = g_attitude.q1, q2 = g_attitude.q2, q3 = g_attitude.q3;
    g_attitude.q0 += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * DT;
    g_attitude.q1 += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * DT;
    g_attitude.q2 += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * DT;
    g_attitude.q3 += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * DT;
    
    //四元数归一化
    norm = sqrtf(g_attitude.q0 * g_attitude.q0 + g_attitude.q1 * g_attitude.q1 + g_attitude.q2 * g_attitude.q2 + g_attitude.q3 * g_attitude.q3);
    g_attitude.q0 /= norm;
    g_attitude.q1 /= norm;
    g_attitude.q2 /= norm;
    g_attitude.q3 /= norm;
    
    //将四元数转换为欧拉角
    g_attitude.roll  = atan2f(2.0f * (g_attitude.q0 * g_attitude.q1 + g_attitude.q2 * g_attitude.q3), 1.0f - 2.0f * (g_attitude.q1 * g_attitude.q1 + g_attitude.q2 * g_attitude.q2)) * RAD_TO_DEG;
    g_attitude.pitch = asinf(2.0f * (g_attitude.q0 * g_attitude.q2 - g_attitude.q3 * g_attitude.q1)) * RAD_TO_DEG;
    g_attitude.yaw   = atan2f(2.0f * (g_attitude.q0 * g_attitude.q3 + g_attitude.q1 * g_attitude.q2), 1.0f - 2.0f * (g_attitude.q2 * g_attitude.q2 + g_attitude.q3 * g_attitude.q3)) * RAD_TO_DEG;
}
    void Attitude_Init(void){
    NEUQ_LCD200_DisplayString(8, 100, "Calibrating IMU...");
    NEUQ_LCD200_DisplayString(8, 130, "Keep still 5s");
    Attitude_Ini();}