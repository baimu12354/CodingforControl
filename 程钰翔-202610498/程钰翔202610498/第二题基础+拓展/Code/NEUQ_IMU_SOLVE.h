
#ifndef _NEUQ_IMU_SOLVE_H_
#define _NEUQ_IMU_SOLVE_H_



typedef struct
{
        float q0;
        float q1;
        float q2;
        float q3;
}quaternion_struct;

extern quaternion_struct quaternion;   // Mahony 姿态四元数
extern float roll;                     // 翻滚角 单位：度
extern float pitch;                    // 俯仰角 单位：度
extern float yaw;                      // 偏航角 单位：度

extern float gyro_x_offset;            // 陀螺仪三轴零偏（上电标定，静止时自动跟踪）
extern float gyro_y_offset;
extern float gyro_z_offset;

void mpu_solve_zero(void);   // 上电静止标定陀螺仪三轴零偏
void mpu_gyro_offset_track(float gx, float gy, float gz, float ax, float ay, float az);  // 静止时自动跟踪零偏

void mpu_mahony_init(void);
void mpu_mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void mpu_get_attitude(void);

#endif 
