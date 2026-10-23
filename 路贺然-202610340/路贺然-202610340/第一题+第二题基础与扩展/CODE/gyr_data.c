#include "NEUQ_lib_include.h"
#include "gyr_data.h"

int16_t ax_raw, ay_raw, az_raw;   // 三轴加速度
int16_t gx_raw, gy_raw, gz_raw;   //三轴角速度

float acc_x_g, acc_y_g, acc_z_g;          //三轴加速度  （物理量 g）
float gyro_x_dps, gyro_y_dps, gyro_z_dps; // 三轴角速度（物理量 °/s）

//================================================================
void MPU6050_Measure_6Axis(void)
{
    NEUQ_MPU6050_read_data();

    ax_raw = mpu6050_acc_x;
    ay_raw = mpu6050_acc_y;
    az_raw = mpu6050_acc_z;

    gx_raw = mpu6050_gyro_x;
    gy_raw = mpu6050_gyro_y;
    gz_raw = mpu6050_gyro_z;

    acc_x_g = MPU6050_acc_transition(ax_raw)*10;    // 原始值 -> m/^2
    acc_y_g = MPU6050_acc_transition(ay_raw)*10;
    acc_z_g = MPU6050_acc_transition(az_raw)*10;

    gyro_x_dps = MPU6050_gyro_transition(gx_raw); // 原始值 -> °/s
    gyro_y_dps = MPU6050_gyro_transition(gy_raw);
    gyro_z_dps = MPU6050_gyro_transition(gz_raw);
}