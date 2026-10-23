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

/* 功能实现见 CODE；这里仅保留初始化、方案切换与非阻塞调度。 */
void User_Main(void)
{
    uint8_t current_scheme = 0;
    uint8_t pending_scheme = 0;
    uint32_t period_start;
    uint32_t last_lcd = 0;
    uint32_t now;
    uint32_t battery_mv = 0;

    NEUQ_Debug_init();
    NEUQ_Delay_init();
    Led_Control_Init();
    Key_Input_Init();
    Battery_Monitor_Init();
    Led_Display_Init();

    period_start = HAL_GetTick();

    while (1)
    {
        now = HAL_GetTick();
//一旦按键按下被被确然为有效值那么久更细此时的pending状态
        if (Key_Input_Pressed())
        {
            pending_scheme = (uint8_t)((pending_scheme + 1U) % 4U);
            //加一取余可以让按键一直在0-3内循环
        }

        /* 周期边界才提交待执行方案；跨过多个周期时也能自动追赶。 */
        if ((uint32_t)(now - period_start) >= APP_PERIOD_MS)
        {
            period_start += ((now - period_start) / APP_PERIOD_MS) * APP_PERIOD_MS;
            //此处的除是取整数个周期，目的是看当前过去了多少个周期，然后赋值给period_start形成完整的周期数
            current_scheme = pending_scheme;
        }

        Led_Control_Update(current_scheme, (uint32_t)(now - period_start));

        if ((uint32_t)(now - last_lcd) >= APP_LCD_REFRESH_MS)
        {
            last_lcd = now;
            battery_mv = Battery_Monitor_ReadMillivolt();
            Led_Display_Refresh(current_scheme, pending_scheme, battery_mv);
        }
    }
}
//整体使用了非阻塞调度