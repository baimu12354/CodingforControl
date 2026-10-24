//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Madgwick 六轴 IMU 姿态解算（已从 C++ 版改成纯 C 版）。
// 原始算法：http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/ （GPL 许可）
//
// 用法：
//   开机      Madgwick_Init(采样频率Hz);   // 采样频率必须和调用周期一致
//   每次采样   Madgwick_UpdateIMU(gx, gy, gz, ax, ay, az);
//   取角       roll = Madgwick_GetRoll();  // 单位：度
//
// 注：原始库还有一个带磁力计的 9 轴版本 update()，本工程 MPU6050 没有磁力计，
//     用不到，所以这里只保留 6 轴版本 updateIMU()。
//=============================================================================================
#ifndef MADGWICK_AHRS_H_
#define MADGWICK_AHRS_H_

// 初始化：sampleFrequency = 调用 Madgwick_UpdateIMU 的频率(Hz)，必须与实际采样周期一致
void Madgwick_Init(float sampleFrequency);

// 设置算法增益 beta（默认 0.1；越大越信任加速度计、响应快但易抖，越小越平滑）
void Madgwick_SetBeta(float beta);

// 六轴 IMU 更新（无磁力计）：每次采样调用一次
//   gx/gy/gz 角速度，单位 rad/s
//   ax/ay/az 加速度，单位 g
void Madgwick_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

// 姿态角（度）：roll/pitch 约 -180~180，yaw 0~360（原库约定，加了 180 偏移）
float Madgwick_GetRoll(void);
float Madgwick_GetPitch(void);
float Madgwick_GetYaw(void);

// 姿态角（弧度）
float Madgwick_GetRollRadians(void);
float Madgwick_GetPitchRadians(void);
float Madgwick_GetYawRadians(void);

#endif
