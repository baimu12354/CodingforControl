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
#include "NEUQ_lib_include.h"

int32_t wave_send_enable = 0;
int32_t current_axis = 0;

void NEUQ_Send_Wave(void)
{
  
    if (!wave_send_enable || imu_calibrating) return;
    switch (current_axis) {
        case 0: printf("%.3f,%.3f\n", raw_accel[0], accel_data[0]); break;
        case 1: printf("%.3f,%.3f\n", raw_gyro[0], gyro_data[0]); break;
        case 2: printf("%.3f,%.3f\n", raw_accel[1], accel_data[1]); break;
        case 3: printf("%.3f,%.3f\n", raw_gyro[1], gyro_data[1]); break;
        case 4: printf("%.3f,%.3f\n", raw_accel[2], accel_data[2]); break;
        case 5: printf("%.3f,%.3f\n", raw_gyro[2], gyro_data[2]); break;
        default: break;
    }
}

void User_Main(void)
{
    NEUQ_Debug_init();
    NEUQ_Delay_init();
    NEUQ_Key_Init();
    NEUQ_LCD200_init();
    NEUQ_LCD200_DisplayString(8, 50, "Starting IMU...");
    uint8_t imu_ok = NEUQ_MPU6050_init();
    NEUQ_Flash_init();
    Madgwick_Init();
    if (!imu_ok) imu_calibrating = 0;
    NEUQ_Menu_Init();
    NEUQ_Menu_Task();
    NEUQ_PIT_init_ms(PIT_CH1, KEY_SCAN_PERIOD_MS);

    while (1) {
        if (imu_ok && NEUQ_IMU_Task()) {
            NEUQ_Send_Wave();
        }
        KeyEventEnum key = NEUQ_Key_EventGet();
        if (key != KEY_EVENT_NONE) {
            NEUQ_Menu_SendKey(key);
        }
        NEUQ_Menu_Task();
    }
}
