#ifndef __GET_ANGLE_H__
#define __GET_ANGLE_H__

#include "../NEUQ_lib_include.h"

#define    Gyro_R    0.0010653f    //数据乘以该参数

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}IMU_Offset_s;

typedef struct {
    float gx_a,gy_a,gz_a;  // 陀螺仪测量值（单位：angle/s）
    float gx_r,gy_r,gz_r;  // 陀螺仪测量值（单位：rad/s）
    float gx_r_old,gy_r_old,gz_r_old;
    float ax, ay, az;      // 加速度计测量值（单位：m/s^2）
    float ax_old, ay_old, az_old;
    float mx, my, mz;      // 地磁计测量值（单位：uT）
    float mx_f,my_f,mz_f;  // 地磁计修正值
    IMU_Offset_s offset;    // 误差项
} imu_data_s;

//  roll          横滚角（围绕x轴转动），单位是°
//  pitch         俯仰角（围绕y轴转动），单位是°
//  yaw           偏航角（围绕z轴转动），单位是°
typedef struct {
    float roll_a;      //角度制
    float roll_a_old;
    float pitch_a;     //角度制
    float pitch_a_old;
    float yaw_a;       //角度制
    float yaw_a_old;
    float yaw_s;     //角度制
    float roll_r;      //弧度制
    float pitch_r;     //弧度制
    float yaw_r;       //弧度制

}EulerAngle_s;

typedef struct {
    imu_data_s data;
    EulerAngle_s angle;
    // Gyro_Bias_Compensator_t gyro_bias_comp;
}imu_Data_All_s;

//四元数结构体
typedef struct
{
    double  q0;
    double q1;
    double q2;
    double q3;
    double q0_old;
    double q1_old;
    double q2_old;
    double q3_old;
} Quaternion;

// typedef struct {
//     float bias_estimate[3];    // 零偏估计 [x,y,z]
//     float bias_correction[3];  // 零偏补偿 [x,y,z]
//     uint32_t stationary_time;  // 静止时间计数
//     uint8_t is_calibrated;     // 是否已校准
// } Gyro_Bias_Compensator_t;

//IMU相关数据总结构体
extern imu_Data_All_s IMU;
extern Quaternion q;

void init_system(void);
void print_angle(void);
void compute_orientation(int j);
void IMU_Offset_Init();

void Madgwick_AHRS_6_DOF_Get_Angle(double beta, uint8_t betaIn_is_true, uint8_t delta_t);
extern double BETA;

void AHRS_6_DOF_Get_Angle(float delta_T);
void MPU6050_get_value(void);
void Madgwick_Update(float ax, float ay, float az,
                     float gx_dps, float gy_dps, float gz_dps,
                     float dt_ms);
#endif
