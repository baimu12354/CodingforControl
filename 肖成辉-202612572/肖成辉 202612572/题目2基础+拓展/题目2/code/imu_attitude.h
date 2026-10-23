#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

// 姿态解算封装：基于 MPU6050 六轴原始数据 + Mahony 四元数互补滤波。
// 解算结果（单位：度）写入 param.roll / param.pitch / param.yaw，供菜单显示。

void Attitude_Init(void);
void Attitude_Calibrate(void);
void Attitude_Update(void);

#endif /* IMU_ATTITUDE_H */
