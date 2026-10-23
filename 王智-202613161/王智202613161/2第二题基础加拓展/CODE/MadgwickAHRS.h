#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <stdint.h>
#include <math.h>

#define sampleFreq  200.0f   // 采样频率 (Hz)
#define betaDef     0.1f     // 算法增益（2 * 比例增益）

// Madgwick 算法结构体
typedef struct {
    float q0;   // 四元数 w
    float q1;   // 四元数 x
    float q2;   // 四元数 y
    float q3;   // 四元数 z
    float beta; // 算法增益
} Madgwick;

// 初始化 Madgwick 结构体
// beta: 算法增益，通常取 betaDef (0.1f) 左右
void Madgwick_Init(Madgwick *m, float beta);

// AHRS 算法更新（使用加速度计和磁力计）
void Madgwick_update(Madgwick *m,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz);

// IMU 算法更新（仅使用加速度计，不使用磁力计）
void Madgwick_updateIMU(Madgwick *m,
                        float gx, float gy, float gz,
                        float ax, float ay, float az);

// 获取欧拉角（单位：弧度）
// 旋转顺序：ZYX，即 Yaw - Pitch - Roll
// 返回的是传感器相对地球的姿态角
float Madgwick_getRoll(const Madgwick *m);   // 横滚角，范围 (-π, π]
float Madgwick_getPitch(const Madgwick *m);  // 俯仰角，范围 [-π/2, π/2]
float Madgwick_getYaw(const Madgwick *m);    // 偏航角，范围 (-π, π]

#endif // MadgwickAHRS_h