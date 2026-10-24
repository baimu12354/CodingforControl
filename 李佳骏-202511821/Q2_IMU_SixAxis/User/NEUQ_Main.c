/*
 * 基于 NEUQ 开源库（GPLv3）。
 * Copyright (c) 2025 NEUQ SmartCar Lab
 * 许可证：https://www.gnu.org/licenses/gpl-3.0.html
 */

/* 第二题：六轴数据处理与姿态解算，独立固件。 */

#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "../CODE/task_imu.h"

void User_Main(void)
{
    Task_IMU_Init();

    while (1)
    {
        Task_IMU_Loop();
    }
}
