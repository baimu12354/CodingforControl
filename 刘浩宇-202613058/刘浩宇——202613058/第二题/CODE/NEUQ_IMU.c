#include "NEUQ_lib_include.h"
#include <math.h>

float filtered_acc_x  = 0.0f;  // 滤波后加速度 X
float filtered_acc_y  = 0.0f;  // 滤波后加速度 Y
float filtered_acc_z  = 0.0f;  // 滤波后加速度 Z
float filtered_gyro_x = 0.0f; // 滤波后角速度 X
float filtered_gyro_y = 0.0f; // 滤波后角速度 Y
float filtered_gyro_z = 0.0f; // 滤波后角速度 Z
float attitude_pitch  = 0.0f; //俯仰角
float attitude_roll   = 0.0f;  //横滚角
float attitude_yaw    = 0.0f;   //偏航角        
static float gyro_offset_x = 0.0f; // 陀螺仪零偏 X
static float gyro_offset_y = 0.0f; // 陀螺仪零偏 Y
static float gyro_offset_z = 0.0f; // 陀螺仪零偏 Z
static uint8_t is_mpu_first_read = 1;//是不是第一次读数

void NEUQ_IMU_Init(void) {
    NEUQ_MPU6050_init();
    NEUQ_Delay_ms(500);
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < 2000; i++) {
        NEUQ_MPU6050_read_data();
        sum_x += MPU6050_gyro_transition(mpu6050_gyro_x);
        sum_y += MPU6050_gyro_transition(mpu6050_gyro_y);
        sum_z += MPU6050_gyro_transition(mpu6050_gyro_z);
        NEUQ_Delay_ms(2); 
    }
    gyro_offset_x = sum_x / 2000.0f;
    gyro_offset_y = sum_y / 2000.0f;
    gyro_offset_z = sum_z / 2000.0f;
}

// 陀螺仪六轴数据采集与滤波任务
void MPU6050_Filter_Task(void) {
    NEUQ_MPU6050_read_data();
    float alpha = (params.filter_alpha >= 0 && params.filter_alpha <= 100) ? (params.filter_alpha / 100.0f) : 0.5f; // 低通滤波系数 α
    float beta = 1.0f - alpha; // 低通滤波系数补数 1-α
    // 获取加速度原始值转换
    float raw_acc_x = MPU6050_acc_transition(mpu6050_acc_x); // 加速度原始值 X（转换后）
    float raw_acc_y = MPU6050_acc_transition(mpu6050_acc_y); // 加速度原始值 Y（转换后）
    float raw_acc_z = MPU6050_acc_transition(mpu6050_acc_z); // 加速度原始值 Z（转换后）
    //获取陀螺仪原始值转换
    float raw_gyro_x = MPU6050_gyro_transition(mpu6050_gyro_x) - gyro_offset_x; // 陀螺仪原始值 X（减零偏后）
    float raw_gyro_y = MPU6050_gyro_transition(mpu6050_gyro_y) - gyro_offset_y; // 陀螺仪原始值 Y（减零偏后）
    float raw_gyro_z = MPU6050_gyro_transition(mpu6050_gyro_z) - gyro_offset_z; // 陀螺仪原始值 Z（减零偏后）
    if (is_mpu_first_read) {
        filtered_acc_x = raw_acc_x;
        filtered_acc_y = raw_acc_y;
        filtered_acc_z = raw_acc_z;
        filtered_gyro_x = raw_gyro_x;
        filtered_gyro_y = raw_gyro_y;
        filtered_gyro_z = raw_gyro_z;
        is_mpu_first_read = 0;       // 标记已初始化
        return;
    }
    // 加速度滤波
    filtered_acc_x = (alpha * raw_acc_x ) + (beta * filtered_acc_x );
    filtered_acc_y = (alpha * raw_acc_y ) + (beta * filtered_acc_y );
    filtered_acc_z = (alpha * raw_acc_z ) + (beta * filtered_acc_z );
    // 陀螺仪滤波
    filtered_gyro_x = (alpha * raw_gyro_x ) + (beta * filtered_gyro_x );
    filtered_gyro_y = (alpha * raw_gyro_y ) + (beta * filtered_gyro_y );
    filtered_gyro_z = (alpha * raw_gyro_z ) + (beta * filtered_gyro_z );
}

#define SAMPLE_DT 0.01f  // 采样周期 10ms
#define COMP_ALPHA 0.98f // 互补滤波系数（陀螺仪权重 98%）
void Attitude_Solve_Task(void) {
    //加速度计反算角度
    float acc_roll = atan2f(filtered_acc_y, filtered_acc_z) * 57.2958f; // 加速度计反算的横滚角
    float acc_pitch = -atan2f(filtered_acc_x, sqrtf(filtered_acc_y * filtered_acc_y + filtered_acc_z * filtered_acc_z)) * 57.2958f; // 加速度计反算的俯仰角
    //互补滤波核心公式
    attitude_roll = 0.98f * (attitude_roll + filtered_gyro_x * 0.01f) + 0.02f * acc_roll;
    attitude_pitch = 0.98f * (attitude_pitch + filtered_gyro_y * 0.01f) + 0.02f * acc_pitch;  
    // Yaw角单纯靠陀螺仪积分
    attitude_yaw += filtered_gyro_z * 0.01f;
}