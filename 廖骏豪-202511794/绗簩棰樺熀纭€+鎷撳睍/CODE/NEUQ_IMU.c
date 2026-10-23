/*
 * NEUQ_IMU.c
 * MPU6050 六轴连续采集 + 六轴各自独立的数字低通滤波
 *
 * 滤波器是用双线性变换法把二阶巴特沃斯低通离散化出来的：
 *   模拟传递函数  H(s) = w0^2 / (s^2 + (w0/Q)s + w0^2)，Q取0.707
 *   代 s = 2fs*(1-z^-1)/(1+z^-1)，其中 w = 2*pi*fc/fs，alpha = sin(w)/(2Q)，得到
 *       b0 = (1-cos w)/2,  b1 = 1-cos w,  b2 = (1-cos w)/2
 *       a0 = 1+alpha,      a1 = -2cos w,  a2 = 1-alpha
 *   全部除以a0归一化之后，按差分方程算：
 *       y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *
 * 为什么不用最简单的一阶 y = y + a*(x-y)：
 *   一阶每倍频程只衰减6dB，想把抖动压下去就得把截止频率降得很低，趋势也跟着变迟钝；
 *   二阶衰减12dB，同样平滑程度下相位滞后更小，"压抖动"和"留趋势"好兼顾一些。
 * 截止频率为什么用Hz而不是a：
 *   截止频率跟采样率没关系，物理意义也明确，改采样率的时候不用重算。
 */
#include "NEUQ_lib_include.h"
#include <math.h>
#include "NEUQ_Setting.h"
#include "NEUQ_IMU.h"

NEUQ_IMU_Data_t g_imu;

// 六个滤波器，[0..2]是加速度XYZ，[3..5]是角速度XYZ
static NEUQ_Biquad_t s_lpf[6];

// 连续读失败这么多次就认为掉线了，500Hz下20次是40ms
#define NEUQ_IMU_ERR_LIMIT      (20U)
static uint16_t s_retry_cnt = 0;

// 置1表示下次采样要用真实数据给六个滤波器预热（开机、校准、改参数之后）
static uint8_t  s_need_prime = 1;

// 算二阶低通系数
static void NEUQ_Biquad_LowPass(NEUQ_Biquad_t *f, float fc_hz, float fs_hz)
{
    const float Q = 0.70710678f;    // 巴特沃斯
    float w, cw, sw, alpha, a0, inv;

    // 截止频率必须低于0.45倍采样率，超过奈奎斯特系数就发散了
    if(!(fc_hz >= 0.1f) || !(fc_hz <= fs_hz * 0.45f))
    {
        fc_hz = fs_hz * 0.20f;
    }

    w     = 2.0f * 3.14159265f * fc_hz / fs_hz;
    cw    = cosf(w);
    sw    = sinf(w);
    alpha = sw / (2.0f * Q);
    a0    = 1.0f + alpha;
    inv   = 1.0f / a0;

    f->b0 = (1.0f - cw) * 0.5f * inv;
    f->b1 = (1.0f - cw) * inv;
    f->b2 = f->b0;
    f->a1 = -2.0f * cw * inv;
    f->a2 = (1.0f - alpha) * inv;
}

// 把历史状态全填成当前值，这样开机不会出现从0慢慢爬上来的一段过渡
static void NEUQ_Biquad_Prime(NEUQ_Biquad_t *f, float v)
{
    f->x1 = v; f->x2 = v;
    f->y1 = v; f->y2 = v;
}

static inline float NEUQ_Biquad_Run(NEUQ_Biquad_t *f, float x)
{
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

float NEUQ_IMU_GetSampleHz(void)
{
    return 1000.0f / (float)NEUQ_IMU_PERIOD_MS;
}

// 把14字节原始数据拆到 g_imu.raw[]
static void NEUQ_IMU_ParseRaw(const uint8_t *p)
{
    g_imu.raw[0] = (int16_t)((p[0]  << 8) | p[1]);   // accel X
    g_imu.raw[1] = (int16_t)((p[2]  << 8) | p[3]);   // accel Y
    g_imu.raw[2] = (int16_t)((p[4]  << 8) | p[5]);   // accel Z
    // p[6] p[7] 是温度，这里不用
    g_imu.raw[3] = (int16_t)((p[8]  << 8) | p[9]);   // gyro X
    g_imu.raw[4] = (int16_t)((p[10] << 8) | p[11]);  // gyro Y
    g_imu.raw[5] = (int16_t)((p[12] << 8) | p[13]);  // gyro Z

    // 同步给库里的全局变量，别的模块和库函数可能还要用
    mpu6050_acc_x  = g_imu.raw[0];
    mpu6050_acc_y  = g_imu.raw[1];
    mpu6050_acc_z  = g_imu.raw[2];
    mpu6050_gyro_x = g_imu.raw[3];
    mpu6050_gyro_y = g_imu.raw[4];
    mpu6050_gyro_z = g_imu.raw[5];
}

uint8_t NEUQ_IMU_Init(void)
{
    uint8_t id = 0;

    // 库函数负责IIC初始化、复位、唤醒、WHO_AM_I检查
    if(NEUQ_MPU6050_init() == 0)
    {
        g_imu.online = 0;
        return 0;
    }

    // 再读一次WHO_AM_I确认通信真的通了，正常值是0x68
    if(NEUQ_Hard_IIC_read_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, MPU6050_WHO_AM_I, &id, 1) == 0)
    {
        g_imu.online = 0;
        return 0;
    }

    // 时钟源选PLL(X轴陀螺)，比内部8MHz振荡器稳，采样周期抖动小一些
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);
    NEUQ_Delay_ms(10);

    // 采样分频0，内部陀螺输出1kHz，我们每2ms读一次正好取到
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 0x00);

    // 内部DLPF给0x01，陀螺188Hz加速度184Hz
    // 故意放这么宽是为了让原始波形保留真实抖动，不然软件滤波前后看不出差别
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, MPU6050_CONFIG, 0x01);
    NEUQ_Delay_ms(1);

    // 量程要和NEUQ_IMU.h里的LSB常数对上：加速度±4g，角速度±2000°/s
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_4G);
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, MPU6050_GYRO_CONFIG,  MPU6050_GYRO_FS_2000);
    NEUQ_Delay_ms(10);

    g_imu.err_cnt = 0;
    g_imu.online  = 1;
    s_retry_cnt   = 0;
    NEUQ_IMU_SetFilterHz(g_setting.acc_lpf_hz, g_setting.gyro_lpf_hz);
    NEUQ_IMU_ResetFilter();

    return 1;
}

// 改截止频率，立刻重算六个滤波器的系数
void NEUQ_IMU_SetFilterHz(float acc_hz, float gyro_hz)
{
    float fs = NEUQ_IMU_GetSampleHz();
    uint8_t i;

    for(i = 0; i < 3; i++)
    {
        NEUQ_Biquad_LowPass(&s_lpf[i], acc_hz, fs);         // 加速度三轴
    }
    for(i = 3; i < 6; i++)
    {
        NEUQ_Biquad_LowPass(&s_lpf[i], gyro_hz, fs);        // 角速度三轴
    }
}

// 用当前最新采样值预热六个滤波器
void NEUQ_IMU_ResetFilter(void)
{
    uint8_t i;
    for(i = 0; i < 3; i++)
    {
        NEUQ_Biquad_Prime(&s_lpf[i],     g_imu.acc_g_raw[i]);
        NEUQ_Biquad_Prime(&s_lpf[3 + i], g_imu.gyro_dps_raw[i]);
        g_imu.acc_g[i]    = g_imu.acc_g_raw[i];
        g_imu.gyro_dps[i] = g_imu.gyro_dps_raw[i];
    }
    s_need_prime = 1;       // 下次采样拿到真实数据后再预热一遍
}

// 读一次传感器 -> 换算 -> 六轴分别滤波
void NEUQ_IMU_Update(void)
{
    uint8_t buf[14];
    uint8_t i;

    // 掉线之后降速重试。库里I2C读失败会printf，如果还按500Hz重试，
    // 日志会把主循环拖得很慢，所以这里大约1秒才试一次。
    if(g_imu.err_cnt > NEUQ_IMU_ERR_LIMIT)
    {
        s_retry_cnt++;
        if(s_retry_cnt >= 500)
        {
            s_retry_cnt = 0;
            if(NEUQ_IMU_Init())
            {
                g_imu.err_cnt = 0;
                g_imu.online  = 1;
            }
        }
        return;
    }

    if(NEUQ_Hard_IIC_read_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR,
                                   MPU6050_ACCEL_XOUT_H, buf, 14) == 0)
    {
        g_imu.err_cnt++;
        if(g_imu.err_cnt > NEUQ_IMU_ERR_LIMIT)
        {
            g_imu.online = 0;
        }
        return;                             // 读失败就沿用上一次的数据
    }

    g_imu.err_cnt = 0;
    g_imu.online  = 1;
    NEUQ_IMU_ParseRaw(buf);

    // 原始计数换算成物理量
    for(i = 0; i < 3; i++)
    {
        g_imu.acc_g_raw[i]    = (float)g_imu.raw[i]     / NEUQ_IMU_ACC_LSB_PER_G;
        g_imu.gyro_dps_raw[i] = (float)g_imu.raw[3 + i] / NEUQ_IMU_GYRO_LSB_PER_DPS;
    }

    // 六轴各自独立滤波
    if(s_need_prime)
    {
        // 第一次采样，或者刚校准完改完参数，把滤波器历史状态置成当前真实值，
        // 这样输出直接等于原始值，屏幕上不会出现从0慢慢爬的怪现象。
        // 注意NEUQ_IMU_ResetFilter()里会把s_need_prime重新置1，所以要在这之后才清标志。
        NEUQ_IMU_ResetFilter();
        s_need_prime = 0;
    }

    for(i = 0; i < 3; i++)
    {
        g_imu.acc_g[i]    = NEUQ_Biquad_Run(&s_lpf[i],     g_imu.acc_g_raw[i]);
        g_imu.gyro_dps[i] = NEUQ_Biquad_Run(&s_lpf[3 + i], g_imu.gyro_dps_raw[i]);
    }

    g_imu.ok_cnt++;
}

// 静止状态下求三轴陀螺零偏，边采样边累加
// 校准结果写进 g_imu.gyro_bias_dps[]，姿态解算的时候先减掉它再积分
void NEUQ_IMU_CalibrateGyro(uint16_t samples)
{
    uint8_t buf[14];
    double sum[3] = {0.0, 0.0, 0.0};
    uint16_t good = 0;
    uint16_t i;
    uint8_t  k;

    if(samples == 0) samples = 1000;

    for(i = 0; i < samples; i++)
    {
        if(NEUQ_Hard_IIC_read_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR,
                                       MPU6050_ACCEL_XOUT_H, buf, 14) == 1)
        {
            int16_t gyro[3];
            gyro[0] = (int16_t)((buf[8]  << 8) | buf[9]);
            gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
            gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);
            for(k = 0; k < 3; k++)
            {
                sum[k] += (double)gyro[k] / (double)NEUQ_IMU_GYRO_LSB_PER_DPS;
            }
            good++;
        }
        NEUQ_Delay_ms(NEUQ_IMU_PERIOD_MS);
    }

    if(good > 0)
    {
        for(k = 0; k < 3; k++)
        {
            g_imu.gyro_bias_dps[k] = (float)(sum[k] / (double)good);
        }
    }

    NEUQ_IMU_ResetFilter();     // 用校准后的数据重新预热
    g_imu.err_cnt = 0;
}
