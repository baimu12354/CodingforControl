/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_Interrupts
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
* 2025-09-27        张瑞        加入外部中断回调
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Main.h"
#include <math.h>
#include "fournumber.h"
#include "keyqueue.h"
#include "flash.h"
#include "NEUQ_Filter.h"

//  uint8_t sample_started = 0;
  uint16_t calibration_count = 0;
// float gyro_x_bias=17.4;
// float gyro_y_bias=-4.5;
// float gyro_z_bias=16.3;

// float gyro_x_bias=118.76;
// float gyro_y_bias=32.62;
// float gyro_z_bias=-12.15;
// float gyro_x_bias=23.30;
// float gyro_y_bias=1.92;
// float gyro_z_bias=-44.84;
float gyro_x_bias=22.797;
float gyro_y_bias=1.546;
float gyro_z_bias=-44.949;

float gyro_x_sum, gyro_y_sum, gyro_z_sum;







// **************************** PIT中断函数 ****************************
void NEUQ_PIT_1_CALLBACK(void){


    flash_count++;
    if(flash_count >= 70) {
        flash_count = 0;
        lcd_fresh = 1;
    }

    float gx, gy, gz;

     GPIO_Ports_ENUM pins[4] = { KEY_UP, KEY_DOWN, KEY_ENTER, KEY_BACK };
    for (uint8_t i = 0U; i < 4; ++i) 
    if (key_read(pins[i], i) ) 
    key_queue_push(i);

    NEUQ_MPU6050_read_data();
    imudata_.acc_x  =(mpu6050_acc_x); 
    imudata_.acc_y = (mpu6050_acc_y); 
    imudata_.acc_z = (mpu6050_acc_z);
    imudata_.gyro_x= ( mpu6050_gyro_x); 
    imudata_.gyro_y =(mpu6050_gyro_y); 
    imudata_.gyro_z =(mpu6050_gyro_z);
    // if (!sample_started) { filter_reset(&imudata_); sample_started = 1; }
    imudata_.acc_x_f =  LowPass(imudata_.acc_x_f, (float)imudata_.acc_x, filter_alfa_acc);
    imudata_.acc_y_f =  LowPass(imudata_.acc_y_f, (float)imudata_.acc_y, filter_alfa_acc);
    imudata_.acc_z_f =  LowPass(imudata_.acc_z_f, (float)imudata_.acc_z, filter_alfa_acc);
    imudata_.gyro_x_f = LowPass(imudata_.gyro_x_f, ((float)imudata_.gyro_x-gyro_x_bias),filter_alfa_gyro);
    imudata_.gyro_y_f = LowPass(imudata_.gyro_y_f, ((float)imudata_.gyro_y-gyro_y_bias),filter_alfa_gyro);
    imudata_.gyro_z_f = LowPass(imudata_.gyro_z_f, ((float)imudata_.gyro_z-gyro_z_bias),filter_alfa_gyro);


// if (calibration_count < 1000) {
//     gyro_x_sum += (float)imudata_.gyro_x;
//     gyro_y_sum += (float)imudata_.gyro_y;
//     gyro_z_sum += (float)imudata_.gyro_z;

//     calibration_count++;

//     if (calibration_count == 1000) {
//         gyro_x_bias = gyro_x_sum / 1000.0f;
//         gyro_y_bias = gyro_y_sum / 1000.0f;
//         gyro_z_bias = gyro_z_sum / 1000.0f;
//     }
// }


    gx = MPU6050_gyro_transition(imudata_.gyro_x-gyro_x_bias );
    gy = MPU6050_gyro_transition(imudata_.gyro_y-gyro_y_bias );
    gz = MPU6050_gyro_transition(imudata_.gyro_z-gyro_z_bias );

    if(fabsf(gz) < 2.0f)
    {
        gz=0;

    }
        if(fabsf(gx) < 2.0f)
    {
        gx=0;

    }
        if(fabsf(gy) < 2.0f)
    {
        gy=0;

    }
    fournumber_update(imudata_.acc_x_f, imudata_.acc_y_f, imudata_.acc_z_f, gx, gy, gz);

    
    imudata_.roll = Roll;
    imudata_.pitch = Pitch;
    imudata_.yaw = Yaw;
    // norm = sqrtf(imudata_.acc_y_f * imudata_.acc_y_f + imudata_.acc_z_f * imudata_.acc_z_f);
    // pitch_acc = atan2f(-imudata_.acc_x_f, norm) * 57.2957795f;
    // roll_acc = atan2f(imudata_.acc_y_f, imudata_.acc_z_f) * 57.2957795f;
    // imudata_.roll = 0.98f * (imudata_.roll + gx * 0.01f) + 0.02f * roll_acc;
    // imudata_.pitch = 0.98f * (imudata_.pitch + gy * 0.01f) + 0.02f * pitch_acc;
    // imudata_.yaw += gz * 0.01f;
}
void NEUQ_PIT_2_CALLBACK(void){

}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
void TS_UART_0_RX_CALLBACK(uint8_t dat){
    (void)dat;
}

void TS_UART_1_RX_CALLBACK(uint8_t dat){
    (void)dat;
}

void TS_UART_2_RX_CALLBACK(uint8_t dat){
    (void)dat;
}

void TS_UART_3_RX_CALLBACK(uint8_t dat){
    (void)dat;
}
// **************************** 串口中断函数 ****************************


// **************************** 外部中断函数 ****************************
void NEUQ_EXTI0_Callback(void)  {

}

void NEUQ_EXTI1_Callback(void)  {

}

void NEUQ_EXTI2_Callback(void)  {

}

void NEUQ_EXTI3_Callback(void)  {

}

void NEUQ_EXTI4_Callback(void)  {

}

void NEUQ_EXTI5_Callback(void)  {

}

void NEUQ_EXTI6_Callback(void)  {

}

void NEUQ_EXTI7_Callback(void)  {

}

void NEUQ_EXTI8_Callback(void)  {

}

void NEUQ_EXTI9_Callback(void)  {

}

void NEUQ_EXTI10_Callback(void) {

}

void NEUQ_EXTI11_Callback(void) {

}

void NEUQ_EXTI12_Callback(void) {

}

void NEUQ_EXTI13_Callback(void) {

}

void NEUQ_EXTI14_Callback(void) {

}

void NEUQ_EXTI15_Callback(void) {

}
// **************************** 外部中断函数 ****************************
