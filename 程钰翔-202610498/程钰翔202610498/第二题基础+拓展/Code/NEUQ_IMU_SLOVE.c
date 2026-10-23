

#include "NEUQ_lib_include.h"
#include <stdbool.h>
#include <math.h>
#include "NEUQ_IMU_SOLVE.h"


void mpu_solve_zero(void)//3000次导致延时太长，改为500次，上电静止时标定零偏
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    for(int i=0;i<500;i++)
    {
         NEUQ_MPU6050_read_data();
        sum_x += MPU6050_gyro_transition(mpu6050_gyro_x);
        sum_y += MPU6050_gyro_transition(mpu6050_gyro_y);
        sum_z += MPU6050_gyro_transition(mpu6050_gyro_z);
        NEUQ_Delay_ms(2);
    }
    gyro_x_offset = sum_x / 500.0f;
    gyro_y_offset = sum_y / 500.0f;
    gyro_z_offset = sum_z / 500.0f;
}

//陀螺仪零偏（标定 + 静止自动跟踪） 
// 六轴（无磁力计）时 yaw 只能由陀螺仪积分得到，残余零偏会让静止时 yaw 缓慢单调漂移。
// 管理三轴零偏：上电静止标定一次，之后只要检测到静止就用泄漏积分把零偏
// 慢慢追到当前读数，使静止时积分速率趋于 0，从而把静止漂移压到几乎为 0。
// 注意：加速度计不做零偏标定（静止时含 1g 重力，减掉会破坏姿态解算的重力参考）。
float gyro_x_offset = 0.0f;
float gyro_y_offset = 0.0f;
float gyro_z_offset = 0.0f;

#define MPU_GYRO_DELTA_TH        (0.35f)    //静止判定：相邻采样角速度变化阈值(°/s)
#define MPU_ACC_NORM_LO          (0.7f)     //静止判定：加速度模长下限(g)
#define MPU_ACC_NORM_HI          (1.3f)     //静止判定：加速度模长上限(g)
#define MPU_GYRO_STATIC_SAMPLES  (20)       //连续静止多少拍后才开始跟踪（20Hz 约 1s）
#define MPU_GYRO_BIAS_GAIN       (0.02f)    //零偏跟踪增益（20Hz 时时间常数约 2.5s）


//静止判据用相邻采样变化量而不是「角速度绝对值」：真实转动会让相邻两拍明显变化，
void mpu_gyro_offset_track(float gx, float gy, float gz, float ax, float ay, float az)
{
    static float last_x = 0.0f, last_y = 0.0f, last_z = 0.0f;
    static uint8_t primed = 0;
    static uint16_t static_cnt = 0;
    float dx, dy, dz, acc_norm;

    if(!primed)   //第一次调用只记录基准，不做判断
    {
        last_x = gx; last_y = gy; last_z = gz;
        primed = 1;
        return;
    }

    dx = gx - last_x;
    dy = gy - last_y;
    dz = gz - last_z;
    last_x = gx; last_y = gy; last_z = gz;

    acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    if(fabsf(dx) < MPU_GYRO_DELTA_TH && fabsf(dy) < MPU_GYRO_DELTA_TH && fabsf(dz) < MPU_GYRO_DELTA_TH
       && acc_norm > MPU_ACC_NORM_LO && acc_norm < MPU_ACC_NORM_HI)
    {
        if(static_cnt < 1000) static_cnt++;
    }
    else if(static_cnt >= 5)
    {
        static_cnt -= 5;   //迅速退出“静止”，避免把真实转动当漂移修正掉
    }
    else
    {
        static_cnt = 0;   //一检测到运动立即清零，避免把真实转动当漂移修正掉
    }

    if(static_cnt > MPU_GYRO_STATIC_SAMPLES)
    {
        gyro_x_offset += MPU_GYRO_BIAS_GAIN * gx;
        gyro_y_offset += MPU_GYRO_BIAS_GAIN * gy;
        gyro_z_offset += MPU_GYRO_BIAS_GAIN * gz;
    }
}

// Mahony 四元数姿态解算
// 参考 Mahony R, Hamel T, Pflimlin J M. Nonlinear complementary filters on the
// special orthogonal group[J]. IEEE TAC, 2008.
// 只用陀螺仪 + 加速度计（六轴），因此可以稳定解算 roll/pitch，yaw 会缓慢漂移。
#define MPU_MAHONY_TWO_KP       (1.0f)      // 比例增益，越大收敛越快但抗抖动越差
#define MPU_MAHONY_TWO_KI       (0.0f)      // 积分增益，用于消除陀螺仪零偏；0 表示关闭
                                            // （零偏统一由 gyro_x/y/z_offset 处理，避免双重补偿）
#define MPU_MAHONY_YAW_CORRECT  (0)         // 加速度计是否参与修正 yaw(体轴 z)：六轴无磁力计填 0（关闭）
#define MPU_DEG_TO_RAD          (0.017453292519943295f)
#define MPU_RAD_TO_DEG          (57.29577951308232f)

quaternion_struct quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
float roll = 0;
float pitch = 0;
float yaw = 0;

static float mahony_integral_x = 0;
static float mahony_integral_y = 0;
static float mahony_integral_z = 0;

//复位姿态，上电或需要重新对齐时调用
void mpu_mahony_init(void)
{
    quaternion.q0 = 1.0f;
    quaternion.q1 = 0.0f;
    quaternion.q2 = 0.0f;
    quaternion.q3 = 0.0f;
    mahony_integral_x = 0;
    mahony_integral_y = 0;
    mahony_integral_z = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
}
 
//1/sqrt(x)，四元数与加速度都需要归一化
static float mahony_inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

//gx/gy/gz 单位为度每秒，ax/ay/az 单位为 g，dt 单位为秒
void mpu_mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recip_norm;
    float half_vx, half_vy, half_vz;
    float half_ex, half_ey, half_ez;
    float qa, qb, qc;

    gx *= MPU_DEG_TO_RAD;
    gy *= MPU_DEG_TO_RAD;
    gz *= MPU_DEG_TO_RAD;

    //加速度数据有效时才用重力方向做修正，否则退化为纯陀螺仪积分
    if(!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
        recip_norm = mahony_inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        //由当前四元数估算重力方向
        half_vx = quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2;
        half_vy = quaternion.q0 * quaternion.q1 + quaternion.q2 * quaternion.q3;
        half_vz = quaternion.q0 * quaternion.q0 - 0.5f + quaternion.q3 * quaternion.q3;

        //实测重力方向与估算重力方向的叉积即为姿态误差
        half_ex = ay * half_vz - az * half_vy;
        half_ey = az * half_vx - ax * half_vz;

        //六轴（无磁力计）下加速度计对 yaw 没有任何参考信息，它对 z 轴的修正只会把
        //倾斜误差、振动/加速度耦合进航向角，造成 yaw 漂移。因此默认关闭该修正，
        //yaw 完全由陀螺仪积分得到（陀螺仪零偏由 mpu_gyro_offset_track 负责消除）。
#if MPU_MAHONY_YAW_CORRECT
        half_ez = ax * half_vy - ay * half_vx;
#else
        half_ez = 0.0f;
#endif

        //积分项抑制陀螺仪零偏
        mahony_integral_x += MPU_MAHONY_TWO_KI * half_ex * dt;
        mahony_integral_y += MPU_MAHONY_TWO_KI * half_ey * dt;
        mahony_integral_z += MPU_MAHONY_TWO_KI * half_ez * dt;
        gx += mahony_integral_x;
        gy += mahony_integral_y;
        gz += mahony_integral_z;

        //比例项把误差反馈回角速度
        gx += MPU_MAHONY_TWO_KP * half_ex;
        gy += MPU_MAHONY_TWO_KP * half_ey;
        gz += MPU_MAHONY_TWO_KP * half_ez;
    }

    //一阶龙格库塔法积分四元数
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = quaternion.q0;
    qb = quaternion.q1;
    qc = quaternion.q2;
    quaternion.q0 += -qb * gx - qc * gy - quaternion.q3 * gz;
    quaternion.q1 +=  qa * gx + qc * gz - quaternion.q3 * gy;
    quaternion.q2 +=  qa * gy - qb * gz + quaternion.q3 * gx;
    quaternion.q3 +=  qa * gz + qb * gy - qc * gx;

    //归一化，防止数值漂移
    recip_norm = mahony_inv_sqrt(quaternion.q0 * quaternion.q0 + quaternion.q1 * quaternion.q1
                               + quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3);
    quaternion.q0 *= recip_norm;
    quaternion.q1 *= recip_norm;
    quaternion.q2 *= recip_norm;
    quaternion.q3 *= recip_norm;
}

//由四元数解算欧拉角，结果写入 roll/pitch/yaw（单位：度）
void mpu_get_attitude(void)
{
    float q0 = quaternion.q0;
    float q1 = quaternion.q1;
    float q2 = quaternion.q2;
    float q3 = quaternion.q3;
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);

    if(sin_pitch > 1.0f)  sin_pitch = 1.0f;
    if(sin_pitch < -1.0f) sin_pitch = -1.0f;

    roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * MPU_RAD_TO_DEG;
    pitch = asinf(sin_pitch) * MPU_RAD_TO_DEG;
    yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * MPU_RAD_TO_DEG;
}
