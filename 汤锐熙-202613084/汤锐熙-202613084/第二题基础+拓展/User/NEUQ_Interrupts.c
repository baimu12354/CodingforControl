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
#include "../code/config.h"
#include "../code/MadgwickAHRS.h"

// **************************** PIT中断函数 ****************************
void NEUQ_PIT_1_CALLBACK(void){     //输出原始 滤波 解算三组全局变量
    static uint8_t filt_init = 0;
    const float DEG2RAD = 0.0174532925f;         // π/180
    float alpha;

    NEUQ_MPU6050_read_data();                    // 读一次六轴（同一次采样）

    // 原始值抄到我们自己的数组（int16，供 VOFA 上传“原始值,滤波值”）
    g_acc[0]  = mpu6050_acc_x;   g_acc[1]  = mpu6050_acc_y;   g_acc[2]  = mpu6050_acc_z;
    g_gyro[0] = mpu6050_gyro_x;  g_gyro[1] = mpu6050_gyro_y;  g_gyro[2] = mpu6050_gyro_z;

    // 换算成物理单位：加速度 -> g，角速度 -> rad/s（六轴显示页 + 解算共用）
    g_acc_g[0] = MPU6050_acc_transition(mpu6050_acc_x);
    g_acc_g[1] = MPU6050_acc_transition(mpu6050_acc_y);
    g_acc_g[2] = MPU6050_acc_transition(mpu6050_acc_z);
    g_gyro_rad[0] = MPU6050_gyro_transition(mpu6050_gyro_x) * DEG2RAD;
    g_gyro_rad[1] = MPU6050_gyro_transition(mpu6050_gyro_y) * DEG2RAD;
    g_gyro_rad[2] = MPU6050_gyro_transition(mpu6050_gyro_z) * DEG2RAD;

    // ---- Madgwick 姿态解算：角速度减零偏(°/s→rad/s)后输入，加速度直接用 g ----
    Madgwick_UpdateIMU(
        g_gyro_rad[0] - g_gyro_bias[0] * DEG2RAD,
        g_gyro_rad[1] - g_gyro_bias[1] * DEG2RAD,
        g_gyro_rad[2] - g_gyro_bias[2] * DEG2RAD,
        g_acc_g[0], g_acc_g[1], g_acc_g[2]);
    g_roll  = Madgwick_GetRoll();
    g_pitch = Madgwick_GetPitch();
    g_yaw   = Madgwick_GetYaw();

    // 低通系数：菜单存的 0~1000 转成 0~1
    alpha = g_cfg.alpha_x1000 / 1000.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    // 首次滤波：用原始值直接初始化，避免从0慢慢爬升
    if (!filt_init) {
        for (int i = 0; i < 3; i++) {
            g_acc_f[i]  = g_acc[i];
            g_gyro_f[i] = g_gyro[i];
        }
        filt_init = 1;
        return;
    }

    // 六轴独立一阶低通：y += alpha * (x - y)
    for (int i = 0; i < 3; i++) {
        g_acc_f[i]  += alpha * (g_acc[i]  - g_acc_f[i]);
        g_gyro_f[i] += alpha * (g_gyro[i] - g_gyro_f[i]);
    }
}
void NEUQ_PIT_2_CALLBACK(void){
    g_upload_tick = 1;      // 只置标志，真正的发送在主循环里做
}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
void TS_UART_0_RX_CALLBACK(uint8_t dat){

}

void TS_UART_1_RX_CALLBACK(uint8_t dat){
    
}

void TS_UART_2_RX_CALLBACK(uint8_t dat){
    
}

void TS_UART_3_RX_CALLBACK(uint8_t dat){
    
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