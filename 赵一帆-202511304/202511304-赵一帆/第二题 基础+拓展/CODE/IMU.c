#include "IMU.h"
#include "Attitude.h"

IMU_Data_t imu = {0};
volatile uint8_t imu_sample_flag = 0;

void IMU_Init(void) {
    NEUQ_MPU6050_init();
    imu.alpha = 0.2f;
    imu.axis  = IMU_AXIS_AX;
    imu.send_enable = 0;
    
    IMU_Flash_t cfg;
    if (NEUQ_Flash_read(IMU_FLASH_ADDR, (uint8_t*)&cfg, sizeof(cfg)) == NEUQ_FLASH_OK) {
        if (cfg.alpha >= 0.05f && cfg.alpha <= 0.9f) imu.alpha = cfg.alpha;
        if (cfg.axis < 6) imu.axis = cfg.axis;
    }
    Attitude_Init();   // 上电静止校准
}

void IMU_Task(void) {
    // 读一次传感器
    NEUQ_MPU6050_read_data();
    imu.ax = MPU6050_acc_transition(mpu6050_acc_x);
    imu.ay = MPU6050_acc_transition(mpu6050_acc_y);
    imu.az = MPU6050_acc_transition(mpu6050_acc_z);
    imu.gx = MPU6050_gyro_transition(mpu6050_gyro_x);
    imu.gy = MPU6050_gyro_transition(mpu6050_gyro_y);
    imu.gz = MPU6050_gyro_transition(mpu6050_gyro_z);

    // 滤波
    imu.filtered[0] = imu.alpha * imu.ax + (1 - imu.alpha) * imu.filtered[0];
    imu.filtered[1] = imu.alpha * imu.ay + (1 - imu.alpha) * imu.filtered[1];
    imu.filtered[2] = imu.alpha * imu.az + (1 - imu.alpha) * imu.filtered[2];
    imu.filtered[3] = imu.alpha * imu.gx + (1 - imu.alpha) * imu.filtered[3];
    imu.filtered[4] = imu.alpha * imu.gy + (1 - imu.alpha) * imu.filtered[4];
    imu.filtered[5] = imu.alpha * imu.gz + (1 - imu.alpha) * imu.filtered[5];

    // 姿态解算
    Attitude_Update();   
}

void IMU_SendToVOFA(void) {
    if (!imu.send_enable) return;
    float raw = 0, flt = 0;
    switch (imu.axis) {
        case IMU_AXIS_AX: raw = imu.ax; flt = imu.filtered[0]; break;
        case IMU_AXIS_AY: raw = imu.ay; flt = imu.filtered[1]; break;
        case IMU_AXIS_AZ: raw = imu.az; flt = imu.filtered[2]; break;
        case IMU_AXIS_GX: raw = imu.gx; flt = imu.filtered[3]; break;
        case IMU_AXIS_GY: raw = imu.gy; flt = imu.filtered[4]; break;
        case IMU_AXIS_GZ: raw = imu.gz; flt = imu.filtered[5]; break;
    }
    printf("%.3f,%.3f\n", raw, flt);
}

void IMU_EditParam(uint8_t row, int8_t dir) {
    switch (row) {
        case 0: imu.axis = (imu.axis + (dir > 0 ? 1 : 5)) % 6; break;
        case 1: 
            if (dir > 0) imu.send_enable = !imu.send_enable; 
            break;
        case 2: 
            imu.alpha += 0.05f * dir;
            if (imu.alpha > 0.9f)  imu.alpha = 0.9f;
            if (imu.alpha < 0.05f) imu.alpha = 0.05f; 
            break; //上下限
    }
}

void IMU_SaveParams(void) {
    IMU_Flash_t cfg;
    cfg.alpha = imu.alpha;
    cfg.axis  = imu.axis;
    NEUQ_Flash_save(IMU_FLASH_ADDR, (uint8_t*)&cfg, sizeof(cfg));
}