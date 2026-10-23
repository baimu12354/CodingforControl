#include "NEUQ_lib_include.h"

extern float pitch;
extern float roll;
extern float yaw;

extern float q0;
extern float q1;
extern float q2;
extern float q3;

extern uint8_t status;

float gyro_bias[3] = {0};  // 陀螺仪零偏 gx/gy/gz，单位 deg/s

#define ZERO_SAMPLES   2000   // 采样次数

// 陀螺仪零偏标定：静止时采样 N 次取平均
void Zero_offset(void){
    float sum[3] = {0, 0, 0};

    status = OFFSET_RUNNING;

    /* 关中断：防止 PIT 里的 MPU_Receive 与本函数同时操作 I2C 总线，
       读到脏数据导致零偏不准（这是之前零偏不稳的根本原因）          */
    __disable_irq();

    for (int i = 0; i < ZERO_SAMPLES; i++){
        NEUQ_MPU6050_read_data();
        sum[0] += MPU6050_gyro_transition(mpu6050_gyro_x);
        sum[1] += MPU6050_gyro_transition(mpu6050_gyro_y);
        sum[2] += MPU6050_gyro_transition(mpu6050_gyro_z);
        NEUQ_Delay_us(2000);
    }

    gyro_bias[0] = sum[0] / ZERO_SAMPLES;
    gyro_bias[1] = sum[1] / ZERO_SAMPLES;
    gyro_bias[2] = sum[2] / ZERO_SAMPLES;

    __enable_irq();

    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    pitch = 0; roll = 0; yaw = 0;

    status = OFFSET_DONE;
}