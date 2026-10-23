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
* 文件名称          NEUQ_Main
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "Imu_State.h"

ImuData g_imu;
uint8_t g_mpu_ok = 0;

void Imu_Sampling_Update(float dt)
{
    float alpha = (float)g_config.alpha_milli / 1000.0f;
    uint8_t i;
    NEUQ_MPU6050_read_data();
    g_imu.raw[0] = MPU6050_acc_transition(mpu6050_acc_x);
    g_imu.raw[1] = MPU6050_acc_transition(mpu6050_acc_y);
    g_imu.raw[2] = MPU6050_acc_transition(mpu6050_acc_z);
    g_imu.raw[3] = MPU6050_gyro_transition(mpu6050_gyro_x);
    g_imu.raw[4] = MPU6050_gyro_transition(mpu6050_gyro_y);
    g_imu.raw[5] = MPU6050_gyro_transition(mpu6050_gyro_z);

    if (!g_imu.initialized)
    {
        for (i = 0; i < 6U; i++) g_imu.filtered[i] = g_imu.raw[i];
        g_imu.initialized = 1;
    }
    else
    {
        for (i = 0; i < 6U; i++)
            g_imu.filtered[i] += alpha * (g_imu.raw[i] - g_imu.filtered[i]);
    }
    Attitude_Solver_Update(dt);
}//这一段过程的本质就是在读取MPU然后进行滤波，主循环会一直进行这种操作不会在任何的页面操作中出现因此不会被切换切换页面所影响

void Imu_Sampling_Init(void)
{
    g_mpu_ok = (uint8_t)(NEUQ_MPU6050_init() == 0U ? 0U : 1U);
    if (g_mpu_ok) Attitude_Solver_Calibrate();
}
