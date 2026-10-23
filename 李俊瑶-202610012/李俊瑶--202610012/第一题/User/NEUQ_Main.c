/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GNU General Public License 条款重新发布和/或修改它
* 许可证版本为 GPL v3.0 或（您选择的）任何更高版本。
*
* 本开源库的发布是希望它能发挥作用，但不提供任何形式的保证，
* 也不保证适销性或适合特定用途。
* 更多细节请参见 GPL。
*
* 您应该在收到本开源库的同时收到一份 GPL 副本。
* 如果没有，请参阅 <https://www.gnu.org/licenses/>。
*
* 额外说明：修改本程序时请保留原作者版权声明。
*
* 文件名称          NEUQ_Main
* 组织名称          东北大学秦皇岛分校智能车实验室
* 版本信息          NEUQ STM32F407VET6 Opensource Library
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞                first version
*********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

void User_Main(void)
{
    NEUQ_Led_Init();
    NEUQ_LCD200_init();
    NEUQ_Menu_Init();
    NEUQ_Key_Init();
    NEUQ_PIT_init_ms(PIT_CH1, 1);

    while (1)
    {
        NEUQ_Led_Task();

        KeyEventEnum key = NEUQ_Key_EventGet();
        if (key == KEY_UP_SHORT)
        {
            uint8_t plan = NEUQ_Led_GetPendingPlan() + 1;
            if (plan > NEUQ_LED_PLAN_COUNT)
            {
                plan = 1;
            }
            NEUQ_Led_SetPendingPlan(plan);
        }
        else if (key == KEY_DOWN_SHORT)
        {
            uint8_t plan = NEUQ_Led_GetPendingPlan();
            if (plan <= 1)
            {
                plan = NEUQ_LED_PLAN_COUNT;
            }
            else
            {
                plan--;
            }
            NEUQ_Led_SetPendingPlan(plan);
        }

        NEUQ_Menu_Task();
    }
}




