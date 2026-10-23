#include "NEUQ_lib_include.h"
#include "Attitude.h"
#include <math.h>

// �����˲���angle = K*(angle + gyro*dt) + (1-K)*acc_angle
#define CMP_K       0.92f   // �����˲�Ȩ�أ�0.90~0.94����Խ��Խ����������
#define GYRO_DZ     1.0f    // yaw ����(deg/s)����ֹ������ٶȵ��ڴ�ֵ���㣬����Ư��
#define YAW_LP      0.05f   // yaw �����ͨϵ����ԽС yaw �仯Խ��
#define RAD2DEG     57.29577951f
static float gx_bias = 0, gy_bias = 0, gz_bias = 0; //陀螺仪零偏
static float att_roll  = 0.0f;
static float att_pitch = 0.0f;
static float att_yaw   = 0.0f;
static float yaw_int   = 0.0f;   // ���ݻ��ֳ�����ԭʼ yaw
static uint32_t last_tick = 0;
static uint8_t  first_sample = 1;

// �����Ѿ��ǵ�ͨ�˲�������ݣ�gz �Ѽ����ϵ���ƫ����
// roll/pitch �����˲���yaw ���ݻ��� + ���� + �����ͨ��
void Attitude_Update(float gx, float gy, float gz, float ax, float ay, float az)
{//====新增：减去陀螺仪零偏====
    gx = gx - gx_bias;
    gy = gy - gy_bias;
    // gz offset handled by dyn_gz_bias in Main (avoid double subtract)
    uint32_t now = NEUQ_Get_Timer_Value_us();
    if (first_sample) {
        first_sample = 0;
        last_tick = now;
        return;
    }

    uint32_t dt_us = (uint32_t)(uint16_t)(now - last_tick);   // ���ϴ�΢�룬�Զ����� 65ms ����
    float dt = (float)dt_us / 1000000.0f;                      // ΢�� -> ��
    last_tick = now;
    if (dt <= 0.0f || dt >= 0.1f) return;

    // yaw ��������ֹ����С���ٶ����㣬yaw ��Ư
    if (fabsf(gz) < GYRO_DZ) gz = 0.0f;

    // ���ٶȼ�ֱ���� roll/pitch���ȣ�
    float acc_roll  = RAD2DEG * atan2f(ay, az);
    float acc_pitch = RAD2DEG * atan2f(-ax, sqrtf(ay*ay + az*az));

    // �����ںϣ�gyro ����(��Ƶ��Ӧ��) + accel �Ƕ�(��Ƶ����׼)
    att_roll  = CMP_K * (att_roll  + gx * dt) + (1.0f - CMP_K) * acc_roll;
    att_pitch = CMP_K * (att_pitch + gy * dt) + (1.0f - CMP_K) * acc_pitch;

    // yaw�����ݻ��� + ���һ�׵�ͨ���� yaw �仯������ƽ����
    yaw_int += gz * dt;
    while (yaw_int >  180.0f) yaw_int -= 360.0f;
    while (yaw_int < -180.0f) yaw_int += 360.0f;

    float yaw_diff = yaw_int - att_yaw;
    while (yaw_diff >  180.0f) yaw_diff -= 360.0f;
    while (yaw_diff < -180.0f) yaw_diff += 360.0f;
    att_yaw += YAW_LP * yaw_diff;
}

uint8_t Attitude_Calibrated(void) { return 1; }
float   Attitude_GetRoll(void)    { return att_roll; }
float   Attitude_GetPitch(void)   { return att_pitch; }
float   Attitude_GetYaw(void)     { return att_yaw; }
void Attitude_Calib(void)
{
    uint16_t i;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    //采集200组陀螺仪数据求平均，校准期间板子必须静止不动！
    for(i = 0; i < 200; i++)
    {
        NEUQ_MPU6050_read_data();
        sum_gx += MPU6050_gyro_transition(mpu6050_gyro_x);
        sum_gy += MPU6050_gyro_transition(mpu6050_gyro_y);
        sum_gz += MPU6050_gyro_transition(mpu6050_gyro_z);
        NEUQ_Delay_ms(5);
    }
    gx_bias = sum_gx / 200.0f;
    gy_bias = sum_gy / 200.0f;
    gz_bias = sum_gz / 200.0f;
}