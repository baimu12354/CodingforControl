#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include <stdint.h>

/*============================================================================
 * 四元数姿态解算（Mahony 互补滤波）对外接口
 *----------------------------------------------------------------------------
 * 坐标系约定（右手系）：
 *   机体系 b：X 前向、Y 右向、Z 向上。模块水平静置时加速度计读数为
 *            Ax≈0, Ay≈0, Az≈+1g（与 MPU6050 常见模块贴装一致）。
 *   导航系 n：X 北、Y 东、Z 上。
 *   四元数 q=[q0,q1,q2,q3]=[w,x,y,z]，表示机体系 -> 导航系的旋转。
 *
 * 欧拉角（Z-Y-X 旋转顺序，航空常用，单位：度）：
 *   Roll(横滚)  绕 X 轴，正值 = 左倾（左低右高）
 *   Pitch(俯仰) 绕 Y 轴，正值 = 抬头（机头向上）
 *   Yaw(航向)   绕 Z 轴，正值 = 绕 Z 轴逆时针（俯视），以校准时刻为 0 的相对航向
 *----------------------------------------------------------------------------
 * 姿态更新周期：由采样周期决定，本工程 SAMPLE_MS=10ms，即 100Hz。
 * 陀螺仪输入单位为 deg/s，加速度计输入单位为 g，内部均转换到 SI 后再计算。
 *============================================================================*/

// 姿态角（单位：度），供界面显示
extern float att_roll;   // 横滚角
extern float att_pitch;  // 俯仰角
extern float att_yaw;    // 航向角（相对初始方向）

// 复位：四元数置单位四元数、零偏清零、积分误差清零
void Attitude_Init(void);

// 设置静态校准得到的陀螺仪零偏（单位 deg/s）
void Attitude_SetGyroBias(float bx, float by, float bz);

// 用加速度计确定初始横滚/俯仰并构造初始四元数，航向归 0（相对参考）
void Attitude_SetInitFromAccel(float ax, float ay, float az);

// 单步姿态更新：gx/gy/gz 为陀螺仪角速度(deg/s)，ax/ay/az 为加速度(g)，dt 为步长(s)
void Attitude_Update(float gx, float gy, float gz,
                     float ax, float ay, float az, float dt);

#endif
