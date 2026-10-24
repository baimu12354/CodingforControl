/* imu_mpu6050.h - MPU6050 接口 */

#ifndef _IMU_MPU6050_H_
#define _IMU_MPU6050_H_

#include "stm32f4xx_hal.h"

/* 六个轴。顺序与屏幕显示、VOFA+ 选轴、滤波参数一一对应。 */
typedef enum {
    IMU_AX = 0,     /* 加速度 X，单位 g */
    IMU_AY,         /* 加速度 Y，单位 g */
    IMU_AZ,         /* 加速度 Z，单位 g */
    IMU_GX,         /* 角速度 X，单位 °/s */
    IMU_GY,         /* 角速度 Y，单位 °/s */
    IMU_GZ,         /* 角速度 Z，单位 °/s */
    IMU_AXIS_COUNT
} imu_axis_t;

/* 一次采样得到的六轴数据与温度 */
typedef struct {
    float acc[3];       /* 三轴加速度，g */
    float gyro[3];      /* 三轴角速度，°/s */
    float temp_c;       /* 温度，摄氏度 */
} imu_sample_t;

uint8_t     IMU_Init(void);                 /* 1 = 找到并配置好传感器，0 = 失败 */
uint8_t     IMU_Read(imu_sample_t *out);    /* 1 = 本次读取有效，0 = 通信失败 */

uint8_t     IMU_GetAddress(void);           /* 探测到的 7 位地址，0 表示未探测到 */
const char *IMU_AxisName(imu_axis_t axis);  /* "AX" / "GX" ... */
const char *IMU_AxisUnit(imu_axis_t axis);  /* "g" / "dps" */

/* 从采样结果里取出某个轴的值 */
float       IMU_AxisValue(const imu_sample_t *s, imu_axis_t axis);

#endif /* _IMU_MPU6050_H_ */
