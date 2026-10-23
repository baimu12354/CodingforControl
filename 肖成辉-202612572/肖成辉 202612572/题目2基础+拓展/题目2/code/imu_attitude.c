#include "imu_attitude.h"
#include "madgwickFilter.h"
#include "NEUQ_lib_include.h"

// 上电时记录的初始偏航角，用于把当前朝向归零。
static float yaw_zero = 0.0f;
static uint8_t yaw_zeroed = 0;

// 陀螺仪零偏（上电静止标定得到），单位 rad/s。
// z 轴即 yaw 轴：无磁力计等绝对参考，零偏会直接造成偏航漂移，必须标定。
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

// 上电静止标定：采样 N 次陀螺仪数据求平均，作为零偏。
// 必须在 PIT（100Hz 定时器）启动之前调用，且调用期间小车保持静止。
void Attitude_Calibrate(void)
{
    const uint16_t N = 500;                  // 采样次数（每次约 2ms，共约 1s）
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    for (uint16_t i = 0; i < N; i++)
    {
        NEUQ_MPU6050_read_data();
        sum_x += mpu6050_gyro_x;
        sum_y += mpu6050_gyro_y;
        sum_z += mpu6050_gyro_z;
        NEUQ_Delay_ms(2);
    }

    gyro_bias_x = MPU6050_gyro_transition((int16_t)(sum_x / N)) * (PI / 180.0f);
    gyro_bias_y = MPU6050_gyro_transition((int16_t)(sum_y / N)) * (PI / 180.0f);
    gyro_bias_z = MPU6050_gyro_transition((int16_t)(sum_z / N)) * (PI / 180.0f);
}

void Attitude_Init(void)
{
    param.roll  = 0.0f;
    param.pitch = 0.0f;
    param.yaw   = 0.0f;

    // 四元数复位为单位四元数（表示“无旋转”的初始姿态）
    q_est.q1 = 1.0f;
    q_est.q2 = 0.0f;
    q_est.q3 = 0.0f;
    q_est.q4 = 0.0f;

    yaw_zero   = 0.0f;
    yaw_zeroed = 0;
}

void Attitude_Update(void)
{
    float ax, ay, az;   // 加速度 (g)
    float gx, gy, gz;   // 角速度 (rad/s)
    float yaw, pitch, roll;

    // 1) 读取三轴原始数据并换算为物理量。
    //    中断里 DataGet() 已调用 NEUQ_MPU6050_read_data()，这里直接用最新值。
    //    默认按 MPU6050 数据手册方向（右手系）不做翻转；若实测某个角度方向反了，
    //    把对应轴取负即可（加速度和角速度要一起改）。
    ax = MPU6050_acc_transition(mpu6050_acc_x);   // g
    ay = MPU6050_acc_transition(mpu6050_acc_y);
    az = MPU6050_acc_transition(mpu6050_acc_z);

    // 2) 陀螺仪量纲 °/s -> rad/s，并减去上电标定的零偏（Mahony 内部按 rad/s 积分）
    gx = MPU6050_gyro_transition(mpu6050_gyro_x) * (PI / 180.0f) - gyro_bias_x;
    gy = MPU6050_gyro_transition(mpu6050_gyro_y) * (PI / 180.0f) - gyro_bias_y;
    gz = MPU6050_gyro_transition(mpu6050_gyro_z) * (PI / 180.0f) - gyro_bias_z;

    // 3) Mahony 互补滤波：陀螺仪积分 + 加速度计（重力方向）校正
    imu_filter(ax, ay, az, gx, gy, gz);

    // 4) 四元数 -> 欧拉角（单位：度）
    eulerAngles(q_est, &roll, &pitch, &yaw);

    // 5) 第一次解算时把当前偏航记为 0°
    if (!yaw_zeroed)
    {
        yaw_zero   = yaw;
        yaw_zeroed = 1;
    }

    param.roll  = roll;
    param.pitch = pitch;
    param.yaw   = yaw - yaw_zero;
}
