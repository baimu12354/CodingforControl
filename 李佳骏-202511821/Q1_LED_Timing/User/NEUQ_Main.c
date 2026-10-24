/*
 * 基于 NEUQ 开源库（GPLv3）。
 * Copyright (c) 2025 NEUQ SmartCar Lab
 * 许可证：https://www.gnu.org/licenses/gpl-3.0.html
 */

/* 第一题：LED 时序控制，独立固件，只用 LCD、按键、LED 和 PIT。 */

#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "../CODE/task_led.h"

void User_Main(void)
{
    Task_LED_Init();

    while (1)
    {
        Task_LED_Loop();
    }
}
