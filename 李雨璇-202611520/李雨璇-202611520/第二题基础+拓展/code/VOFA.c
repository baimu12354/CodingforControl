#include "NEUQ_lib_include.h"

float ax;
float ay;
float az;
float gx;
float gy;
float gz;

uint8_t mpu_new_flag = 0;

extern Filter_Param *axis[6];
extern uint8_t axis_index;
extern float axis_send;
extern float gyro_bias[3];

//接收陀螺仪数据
void MPU_Receive(void) {
    NEUQ_MPU6050_read_data();

    Filter_Data(&A_X, mpu6050_acc_x);
    Filter_Data(&A_Y, mpu6050_acc_y);
    Filter_Data(&A_Z, mpu6050_acc_z);
    Filter_Data(&G_X, mpu6050_gyro_x);
    Filter_Data(&G_Y, mpu6050_gyro_y);
    Filter_Data(&G_Z, mpu6050_gyro_z);

    ax = MPU6050_acc_transition(A_X.history);
    ay = MPU6050_acc_transition(A_Y.history);
    az = MPU6050_acc_transition(A_Z.history);
    gx = MPU6050_gyro_transition(G_X.history) - gyro_bias[0];
    gy = MPU6050_gyro_transition(G_Y.history) - gyro_bias[1];
    gz = MPU6050_gyro_transition(G_Z.history) - gyro_bias[2];
}

//将数据发送给VOFA+
void VOFA_send(void) {
    if (axis_send == 1){
        switch (axis_index){
            case 0:
              printf("%f,%f\r\n",ax,MPU6050_acc_transition(mpu6050_acc_x));
              break;
            case 1:
              printf("%f,%f\r\n",ay,MPU6050_acc_transition(mpu6050_acc_y));
              break;
            case 2:
              printf("%f,%f\r\n",az,MPU6050_acc_transition(mpu6050_acc_z));
              break;
            case 3:
              printf("%f,%f\r\n",gx,MPU6050_gyro_transition(mpu6050_gyro_x) - gyro_bias[0]);
              break;
            case 4:
              printf("%f,%f\r\n",gy,MPU6050_gyro_transition(mpu6050_gyro_y) - gyro_bias[1]);
              break;
            case 5:
              printf("%f,%f\r\n",gz,MPU6050_gyro_transition(mpu6050_gyro_z) - gyro_bias[2]);
              break;
            default:
              break;
        }
    }
}

// 判断是否需要发送
uint8_t Send_Need(float a){
  if (a == 0) return 0;
  else return 1;
}