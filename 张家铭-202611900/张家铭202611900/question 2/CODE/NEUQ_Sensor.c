#include "NEUQ_lib_include.h"
#include "stdio.h"

float sensor_raw[6];
float sensor_filt[6];

void NEUQ_Sensor_Init(void){
    NEUQ_MPU6050_init();
}

static void Sensor_Send(void){
    int axis=params.send_axis;
    if(axis<0 || axis>5) axis=0;

    char send_buf[64];
    int n = snprintf(send_buf, sizeof(send_buf), "%.3f,%.3f\r\n",
                     sensor_raw[axis], sensor_filt[axis]);
    if(n > 0){
        if(n >= (int)sizeof(send_buf)) n = (int)sizeof(send_buf) - 1;
        NEUQ_Uart_write_data(UART_0, (uint8_t*)send_buf, (uint32_t)n);
    }
}

void NEUQ_Sensor_Task(void){
    NEUQ_MPU6050_read_data();
    sensor_raw[0]=MPU6050_acc_transition(mpu6050_acc_x);
    sensor_raw[1]=MPU6050_acc_transition(mpu6050_acc_y);
    sensor_raw[2]=MPU6050_acc_transition(mpu6050_acc_z);
    // 减去姿态模块标定好的零偏，静止时角速度显示为 0
    sensor_raw[3]=MPU6050_gyro_transition(mpu6050_gyro_x)-gyro_bias[0];
    sensor_raw[4]=MPU6050_gyro_transition(mpu6050_gyro_y)-gyro_bias[1];
    sensor_raw[5]=MPU6050_gyro_transition(mpu6050_gyro_z)-gyro_bias[2];

    for(int i = 0; i < 3; i++){
        if(params.acc_filter_k <= 0)
            sensor_filt[i] = sensor_raw[i];
        else{
            float k = params.acc_filter_k / 100.0f;
            sensor_filt[i] = sensor_filt[i] * (1.0f - k) + sensor_raw[i] * k;
        }
    }

    for(int i = 3; i < 6; i++){
        if(params.gyro_filter_k <= 0)
            sensor_filt[i] = sensor_raw[i];
        else{
            float k = params.gyro_filter_k / 100.0f;
            sensor_filt[i] = sensor_filt[i] * (1.0f - k) + sensor_raw[i] * k;
        }
    }

    static uint32_t last_send=0;
    if(params.send_enable && NEUQ_Get_Timer_Value_ms()-last_send>=10){
        last_send=NEUQ_Get_Timer_Value_ms();
        Sensor_Send();
    }
}
