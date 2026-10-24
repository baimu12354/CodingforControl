/* imu_mpu6050.c - MPU6050 驱动 */

#include "imu_mpu6050.h"
#include "board_config.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

/* 陀螺仪挂在主板的 P3 座子上：PA8 = SCL，PC9 = SDA，正好是库里的 Hard_IIC_3。
 * 如果实测发现模块不在这一路，只改这一行即可。 */
#define IMU_IIC_PORT        (Hard_IIC_3)
#define IMU_IIC_SPEED_HZ    (400000U)

static uint8_t s_addr7 = 0;     /* 7 位器件地址，0 表示尚未探测到 */

static uint8_t IMU_WriteReg(uint8_t reg, uint8_t value)
{
    return NEUQ_Hard_IIC_write_8bit_reg(IMU_IIC_PORT, (uint8_t)(s_addr7 << 1), reg, value);
}

static uint8_t IMU_ReadReg(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    return NEUQ_Hard_IIC_read_8bit_reg(IMU_IIC_PORT, (uint8_t)(s_addr7 << 1), reg, buffer, length);
}

/* WHO_AM_I 固定返回 0x68，与 AD0 无关 */
static uint8_t IMU_ProbeAddress(uint8_t addr7)
{
    uint8_t id = 0;

    s_addr7 = addr7;
    if (IMU_ReadReg(MPU6050_WHO_AM_I, &id, 1) != 1U)
    {
        return 0;
    }
    return (id == 0x68U) ? 1U : 0U;
}

/* 自动探测 0x68/0x69，配置量程与 DLPF */
uint8_t IMU_Init(void)
{
    s_addr7 = 0;

    NEUQ_Hard_IIC_init(IMU_IIC_PORT, IMU_IIC_SPEED_HZ);
    NEUQ_Delay_ms(50);

    /* 先试常见接法（AD0 接 GND），不行再试 AD0 接 VCC */
    if (IMU_ProbeAddress(T2_MPU_ADDR_PRIMARY) == 0U)
    {
        if (IMU_ProbeAddress(T2_MPU_ADDR_ALTERNATE) == 0U)
        {
            s_addr7 = 0;
            return 0;
        }
    }

    /* 复位后重新配置，保证从上电到现在的状态是确定的 */
    IMU_WriteReg(MPU6050_PWR_MGMT_1, 0x80);
    NEUQ_Delay_ms(100);
    IMU_WriteReg(MPU6050_PWR_MGMT_1, 0x00);     /* 退出睡眠，选内部时钟 */
    NEUQ_Delay_ms(10);
    IMU_WriteReg(MPU6050_SMPLRT_DIV,  0x00);    /* 采样率 = 1 kHz / (1+0) */
    IMU_WriteReg(MPU6050_CONFIG,      0x03);    /* 数字低通 44 Hz */
    IMU_WriteReg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_8G);    /* ±8 g */
    IMU_WriteReg(MPU6050_GYRO_CONFIG,  MPU6050_GYRO_FS_2000);   /* ±2000°/s */

    return 1;
}

/* 14 字节突发读，六轴同一时刻 */
uint8_t IMU_Read(imu_sample_t *out)
{
    uint8_t  raw[14];
    int16_t  acc_x, acc_y, acc_z;
    int16_t  gyro_x, gyro_y, gyro_z;
    int16_t  temp_raw;

    if (out == 0)
    {
        return 0;
    }
    if (s_addr7 == 0U)
    {
        return 0;
    }
    if (IMU_ReadReg(MPU6050_ACCEL_XOUT_H, raw, 14) != 1U)
    {
        return 0;
    }

    acc_x  = (int16_t)(((uint16_t)raw[0]  << 8) | raw[1]);
    acc_y  = (int16_t)(((uint16_t)raw[2]  << 8) | raw[3]);
    acc_z  = (int16_t)(((uint16_t)raw[4]  << 8) | raw[5]);
    temp_raw = (int16_t)(((uint16_t)raw[6] << 8) | raw[7]);
    gyro_x = (int16_t)(((uint16_t)raw[8]  << 8) | raw[9]);
    gyro_y = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
    gyro_z = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);

    out->acc[0]  = (float)acc_x  / T2_ACC_LSB_PER_G;
    out->acc[1]  = (float)acc_y  / T2_ACC_LSB_PER_G;
    out->acc[2]  = (float)acc_z  / T2_ACC_LSB_PER_G;
    out->gyro[0] = (float)gyro_x / T2_GYRO_LSB_PER_DPS;
    out->gyro[1] = (float)gyro_y / T2_GYRO_LSB_PER_DPS;
    out->gyro[2] = (float)gyro_z / T2_GYRO_LSB_PER_DPS;
    out->temp_c  = (float)temp_raw / 340.0f + 36.53f;

    return 1;
}

uint8_t IMU_GetAddress(void)
{
    return s_addr7;
}

const char *IMU_AxisName(imu_axis_t axis)
{
    switch (axis)
    {
        case IMU_AX: return "AX";
        case IMU_AY: return "AY";
        case IMU_AZ: return "AZ";
        case IMU_GX: return "GX";
        case IMU_GY: return "GY";
        case IMU_GZ: return "GZ";
        default:     return "??";
    }
}

const char *IMU_AxisUnit(imu_axis_t axis)
{
    return (axis <= IMU_AZ) ? "g" : "dps";
}

float IMU_AxisValue(const imu_sample_t *s, imu_axis_t axis)
{
    if (s == 0)
    {
        return 0.0f;
    }
    if (axis <= IMU_AZ)
    {
        return s->acc[axis];
    }
    if (axis <= IMU_GZ)
    {
        return s->gyro[axis - IMU_GX];
    }
    return 0.0f;
}
