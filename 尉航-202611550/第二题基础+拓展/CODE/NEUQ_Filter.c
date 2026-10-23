#include "NEUQ_Filter.h"


float gyro_bias_deg[3] = {0,0,0};      // 三轴零偏，单位°/s
float raw[6];
float filtered[6];
kalman_struct kf[6];


void kalman_init(kalman_struct *kalman_lcw, float init_x, float init_p){
    kalman_lcw->x = init_x;          // 初始值（陀螺仪给0，静止加速度z轴可给1.0）
    kalman_lcw->p = init_p;          // 初始协方差，一般给5
    kalman_lcw->A = 1;
    kalman_lcw->H = 1;
    kalman_lcw->q = 0.1f;            // 过程噪声 
    kalman_lcw->r = 10.0f;          // 测量噪声 r越大曲线越平滑，响应越慢
}

float kalman_filter(kalman_struct *kalman_lcw, float measure){
    /* Predict */
    kalman_lcw->x = kalman_lcw->A * kalman_lcw->x;
    kalman_lcw->p = kalman_lcw->A * kalman_lcw->A * kalman_lcw->p + kalman_lcw->q;
    /* Update */
    kalman_lcw->gain = kalman_lcw->p * kalman_lcw->H /
                       (kalman_lcw->p * kalman_lcw->H * kalman_lcw->H + kalman_lcw->r);
    kalman_lcw->x = kalman_lcw->x + kalman_lcw->gain * (measure - kalman_lcw->H * kalman_lcw->x);
    kalman_lcw->p = (1 - kalman_lcw->gain * kalman_lcw->H) * kalman_lcw->p;
    return kalman_lcw->x;
}

void Filter_SixAxis(void){
    NEUQ_MPU6050_read_data();
    raw[0] = MPU6050_acc_transition(mpu6050_acc_x);
    raw[1] = MPU6050_acc_transition(mpu6050_acc_y);
    raw[2] = MPU6050_acc_transition(mpu6050_acc_z);
    raw[3] = MPU6050_gyro_transition(mpu6050_gyro_x);
    raw[4] = MPU6050_gyro_transition(mpu6050_gyro_y);
    raw[5] = MPU6050_gyro_transition(mpu6050_gyro_z);

    for(uint8_t i = 0; i < 6; i++){
        filtered[i] = kalman_filter(&kf[i], raw[i]);
    }
}

//应用滤波参数
void Filter_ApplyParams(float q, float r){
    for(uint8_t i = 0; i < 6; i++){
        if(q > 0.0f) kf[i].q = q;
        if(r > 0.0f) kf[i].r = r;
    }
}


//陀螺仪静态零偏校准
// 上电时板子保持静止，调用一次（约0.4秒）
void Gyro_Bias_Calibration(void){
    int32_t sum[3] = {0,0,0};
    const int N = 200;
    for(int i = 0; i < N; i++){
        NEUQ_MPU6050_read_data();
        sum[0] += mpu6050_gyro_x;
        sum[1] += mpu6050_gyro_y;
        sum[2] += mpu6050_gyro_z;
        NEUQ_Delay_ms(2);
    }
    gyro_bias_deg[0] = MPU6050_gyro_transition((float)sum[0] / N);
    gyro_bias_deg[1] = MPU6050_gyro_transition((float)sum[1] / N);
    gyro_bias_deg[2] = MPU6050_gyro_transition((float)sum[2] / N);
}


// 全局姿态四元数：初始(1,0,0,0) = 水平朝前
float q0=1, q1=0, q2=0, q3=0;
float roll_angle, pitch_angle, yaw_angle;


// 陀螺仪积分 + 加速度计纠偏
// gx/gy/gz：rad/s（已减零偏）；ax/ay/az：滤波后的加速度，单位g
void Quat_Update(float gx, float gy, float gz,float ax, float ay, float az, float dt){
    // 1) 用当前四元数预测"重力方向"（机体坐标系下）
    float vx = 2*(q1*q3 - q0*q2);
    float vy = 2*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 2) 加速度计实测方向归一化（n 保留原始模长，供运动检测用）
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if(n < 1e-6f) return;
    ax /= n;  ay /= n;  az /= n;

    // 3) 叉积误差：加速度计方向 与 预测重力方向 的夹角误差
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;

    // 4) 运动检测：|加速度模长-1g| 越大，加速度计越不可信，kp 越小
    float kp = 1.0f;
    if(fabsf(n - 1.0f) > 0.2f) kp = 0.1f;   // 剧烈晃动：修正强度降为1/10

    gx += kp*ex;      // 修正 roll
    gy += kp*ey;      // 修正 pitch

    // 5) 四元数一阶积分
    q0 = q0 + 0.5f*dt*(-q1*gx - q2*gy - q3*gz);
    q1 = q1 + 0.5f*dt*( q0*gx - q3*gy + q2*gz);
    q2 = q2 + 0.5f*dt*( q3*gx + q0*gy - q1*gz);
    q3 = q3 + 0.5f*dt*(-q2*gx + q1*gy + q0*gz);

    // 6) 四元数归一化（题目要求）
    n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0/=n;  q1/=n;  q2/=n;  q3/=n;
}


// 四元数变欧拉角（度）
void Quat_ToEuler(void){
    roll_angle  = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 57.29578f;
    float arg = 2*(q0*q2 - q3*q1);
    if(arg >  1.0f) arg =  1.0f;
    if(arg < -1.0f) arg = -1.0f;
    pitch_angle = asinf(arg) * 57.29578f;
    yaw_angle   = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 57.29578f;
}
