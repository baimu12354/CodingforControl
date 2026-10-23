#include "NEUQ_lib_include.h"

void VOFA_Send_Task(void) {
    if (params.send_enable != 1) return;
    float raw_val = 0.0f;        // 所选轴的原始值（未滤波）
    float filtered_val = 0.0f;   // 所选轴的滤波后值
    switch (params.wave_axis) {
        case 0:
            raw_val = MPU6050_acc_transition(mpu6050_acc_x);
            filtered_val = filtered_acc_x;
            break;
        case 1:
            raw_val = MPU6050_acc_transition(mpu6050_acc_y);
            filtered_val = filtered_acc_y;
            break;
        case 2:
            raw_val = MPU6050_acc_transition(mpu6050_acc_z);
            filtered_val = filtered_acc_z;
            break;
        case 3:
            raw_val = MPU6050_gyro_transition(mpu6050_gyro_x);
            filtered_val = filtered_gyro_x;
            break;
        case 4:
            raw_val = MPU6050_gyro_transition(mpu6050_gyro_y);
            filtered_val = filtered_gyro_y;
            break;
        case 5:
            raw_val = MPU6050_gyro_transition(mpu6050_gyro_z);
            filtered_val = filtered_gyro_z;
            break;
        default: 
        return;
    }
    printf("%.3f,%.3f\n", raw_val, filtered_val);
}