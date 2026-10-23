#include "NEUQ_lib_include.h"
#include "Attitude.h"

typedef enum 
{
    MPU_Init = 0,
    MPU_Wait,
    MPU_Read,
    MPU_Error
}MPU_State;
uint8_t upload_enable = OFF;
static volatile MPU_State state = MPU_Init;
static volatile uint32_t last_time;
static volatile uint32_t error_time;
static uint32_t attitude_last_sample_ms;
float raw[6];
float filterd[6];
uint32_t sample_time = 10;
uint8_t filter_select = 0;
static volatile uint8_t fist_sample = 1;
float mpu_filter_alpha = 0.1f;
static uint16_t error_max = 1000;
volatile uint32_t timer_ms = 0;
uint32_t TIMER_GET_MS(void)
{
    return timer_ms;
}
static void VOFA_SendSelected()
{
   if(filter_select >= 6)
   {
    return;
   }
    float values[2];
    uint8_t tail[4] = {
        0x00, 0x00, 0x80, 0x7F
    };
    values[0] = raw[filter_select];
    values[1] = filterd[filter_select];
    

    uint8_t *p = (uint8_t *)values;
    for (uint32_t i = 0; i < sizeof(values); i++) {
        NEUQ_Uart_write_byte(UART_0, p[i]);
    }


    for (uint32_t i = 0; i < sizeof(tail); i++) {
        NEUQ_Uart_write_byte(UART_0, tail[i]);
    }
}
static uint8_t MPU6050_Read_flag(void)
{
    uint8_t bytes[14];
    /* Only publish a complete successful burst; WHO_AM_I is not a data check. */
    if (NEUQ_Hard_IIC_read_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR,
                                   MPU6050_ACCEL_XOUT_H, bytes, 14) != 1U)
        return 0U;
    mpu6050_acc_x = (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    mpu6050_acc_y = (int16_t)(((uint16_t)bytes[2] << 8) | bytes[3]);
    mpu6050_acc_z = (int16_t)(((uint16_t)bytes[4] << 8) | bytes[5]);
    mpu6050_gyro_x = (int16_t)(((uint16_t)bytes[8] << 8) | bytes[9]);
    mpu6050_gyro_y = (int16_t)(((uint16_t)bytes[10] << 8) | bytes[11]);
    mpu6050_gyro_z = (int16_t)(((uint16_t)bytes[12] << 8) | bytes[13]);
    MPU6050_Data_get.Temperature =
        (int16_t)(((uint16_t)bytes[6] << 8) | bytes[7]) / 340.0f + 36.53f;
    return 1U;
}
static void MPU_filter_Task(void)
{
    uint32_t sample_ms;
    if(!MPU6050_Read_flag())
    {
        state = MPU_Error;
        Attitude_OnSensorError();
        error_time = TIMER_GET_MS();
        return;
    }
    sample_ms = TIMER_GET_MS();
    raw[0] = MPU6050_acc_transition(mpu6050_acc_x);
    raw[1] = MPU6050_acc_transition(mpu6050_acc_y);
    raw[2] = MPU6050_acc_transition(mpu6050_acc_z);
    raw[3] = MPU6050_gyro_transition(mpu6050_gyro_x);
    raw[4] = MPU6050_gyro_transition(mpu6050_gyro_y);
    raw[5] = MPU6050_gyro_transition(mpu6050_gyro_z);

    /* Runs on every valid sample, regardless of page, upload or filter alpha. */
    Attitude_ProcessSample(raw, (float)(uint32_t)(sample_ms - attitude_last_sample_ms) * 0.001f);
    attitude_last_sample_ms = sample_ms;

    uint8_t i;
    for (i = 0; i < 6;i++)
    {
        if (fist_sample)
        {
            filterd[i] = raw[i];
           
        }
        else
        {
            filterd[i] += mpu_filter_alpha * (raw[i] - filterd[i]);
        }
    }
    fist_sample = 0;
    ////VOFA上传函数；
    if (upload_enable) {
    VOFA_SendSelected();
    }
}
void MPU6050_Service(void)
{
    if (state != MPU_Init)
    {
        return;
    }

    if (NEUQ_MPU6050_init() == 0)
    {
        error_time = TIMER_GET_MS();
        Attitude_OnSensorError();
        state = MPU_Error;
    }
    else
    {
        fist_sample = 1;
        Attitude_Init();
        attitude_last_sample_ms = TIMER_GET_MS();
        last_time = TIMER_GET_MS();

        state = MPU_Wait;
    }
}
void MPU6050_Task(void)
{
    uint32_t now = TIMER_GET_MS();

    switch (state)
    {
        case MPU_Init:
        {
            break;
        }
        case MPU_Wait:
        {
            if((uint32_t)(now - last_time) >= sample_time)
            {
                 last_time = now;
                 MPU_filter_Task();
            }
            break;
        }
        case MPU_Error:
        {
            if((uint32_t)(now - error_time) >= error_max)
            {
                state = MPU_Init;
            }
            break;
        }
        default:
        {   
            state = MPU_Init;
        }
    }
}
