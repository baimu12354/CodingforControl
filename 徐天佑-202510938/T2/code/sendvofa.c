
#include "NEUQ_Main.h"












void sendvofa(void)
{
    char frame[64];
    int len;
    float raw_value;
    float filter_value;

    switch (vofa_axis) {
        case MENU_FILTER_AX:
            raw_value = MPU6050_acc_transition(imudata_.acc_x);
            filter_value = MPU6050_acc_transition(imudata_.acc_x_f);
            break;

        case MENU_FILTER_AY:
            raw_value = MPU6050_acc_transition(imudata_.acc_y);
            filter_value = MPU6050_acc_transition(imudata_.acc_y_f);
            break;

        case MENU_FILTER_AZ:
            raw_value = MPU6050_acc_transition(imudata_.acc_z);
            filter_value = MPU6050_acc_transition(imudata_.acc_z_f);
            break;

        case MENU_FILTER_GX:
            raw_value = MPU6050_gyro_transition(imudata_.gyro_x);
            filter_value = MPU6050_gyro_transition(imudata_.gyro_x_f);
            break;

        case MENU_FILTER_GY:
            raw_value = MPU6050_gyro_transition(imudata_.gyro_y);
            filter_value = MPU6050_gyro_transition(imudata_.gyro_y_f);
            break;

        case MENU_FILTER_GZ:
            raw_value = MPU6050_gyro_transition(imudata_.gyro_z);
            filter_value = MPU6050_gyro_transition(imudata_.gyro_z_f);
            break;

        default:
            return;
    }

    len = sprintf(frame, "data:%.3f,%.3f\n",raw_value, filter_value);

    NEUQ_Uart_write_data(UART_0,(uint8_t *)frame,(uint16_t)len);
}