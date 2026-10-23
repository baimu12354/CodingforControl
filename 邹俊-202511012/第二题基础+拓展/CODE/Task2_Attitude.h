/*********************************************************************************************************************
* 文件名称          Task2_Attitude
* 功能说明          第二题挑战部分：四元数姿态解算（Mahony 互补滤波）+ 陀螺仪静置零偏校准
*
*                   坐标系：按 MPU6050 模块丝印方向，X 前、Y 左、Z 上（右手系），水平静置时加速度计 Z 轴读 +1 g
*                   欧拉角（ZYX 顺序，单位：度）：
*                     横滚 roll  绕 X 轴转，右手定则为正（Y 轴那一侧抬起为正）
*                     俯仰 pitch 绕 Y 轴转，右手定则为正（X 轴前端向下为正）
*                     航向 yaw   绕 Z 轴转，右手定则为正（从上往下看逆时针为正），以校准完成时的朝向为 0
*
*                   为什么要"融合"：
*                   · 只用陀螺仪积分：短时间很准，但零偏一直积分，角度会越漂越远
*                   · 只用加速度计算角度：不漂，但一晃动就乱跳，而且算不出航向
*                   · Mahony：以陀螺仪积分为主，再用加速度计测到的重力方向慢慢把横滚、俯仰"拉回"正确值
*                   重力只能修正横滚和俯仰；航向没有绝对参考（六轴没有磁力计），只能靠零偏校准减小漂移。
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#ifndef _TASK2_ATTITUDE_H_
#define _TASK2_ATTITUDE_H_

#include <stdint.h>

//=============================================== 姿态 ===============================================
typedef struct {
    float q0, q1, q2, q3;       // 姿态四元数（模长始终保持为 1）
    float roll, pitch, yaw;     // 由四元数换算出的欧拉角，单位度
} T2_Attitude_Struct;

void T2_Attitude_reset(T2_Attitude_Struct *att, float ax, float ay, float az);  // 用重力方向算出初始姿态，航向置 0
void T2_Attitude_update(T2_Attitude_Struct *att, float gx, float gy, float gz,  // gx gy gz：去掉零偏的角速度 rad/s
                        float ax, float ay, float az, float dt);                // ax ay az：加速度 g；dt：周期 s
void T2_Attitude_zero_yaw(T2_Attitude_Struct *att);                             // 横滚俯仰不变，航向清零

//============================================ 零偏校准 ============================================
typedef enum {
    T2_CALIB_IDLE = 0,          // 还没开始
    T2_CALIB_RUNNING,           // 正在采集，请保持静止
    T2_CALIB_DONE,              // 完成，gyro_bias 可用
} T2_Calib_State_Enum;

typedef struct {
    uint8_t  state;             // 见 T2_Calib_State_Enum
    uint16_t count;             // 已采集的样本数
    uint16_t restarts;          // 因为被碰到而重新开始的次数
    float    sum_g[3], sum_a[3];        // 角速度 / 加速度累加和
    float    min_g[3], max_g[3];        // 角速度最小 / 最大值，用来判断有没有被碰到
    float    gyro_bias[3];      // 结果：三轴角速度零偏 dps
    float    accel_mean[3];     // 结果：校准期间平均加速度 g，用来算初始姿态
} T2_Calib_Struct;

void    T2_Calib_start(T2_Calib_Struct *c);
uint8_t T2_Calib_feed(T2_Calib_Struct *c, const float gyro_dps[3], const float accel_g[3]);   // 送入一组样本，返回状态

#endif
