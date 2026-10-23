#ifndef IMU_SAMPLING_H
#define IMU_SAMPLING_H

/* MPU6050 初始化、六轴采样和一阶低通滤波。 */
void Imu_Sampling_Init(void);
void Imu_Sampling_Update(void);

#endif /* IMU_SAMPLING_H */
