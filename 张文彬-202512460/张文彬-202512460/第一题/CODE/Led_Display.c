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


static void App_DrawStaticPage(void)
{
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(10, 10,  "LED TIMING MENU");
    NEUQ_LCD200_DisplayString(10, 55,  "Current :");
    NEUQ_LCD200_DisplayString(10, 90,  "Pending :");
    NEUQ_LCD200_DisplayString(10, 125, "Battery :");
    NEUQ_LCD200_DisplayString(10, 175, "K0: select next plan");
    NEUQ_LCD200_DisplayString(10, 205, "New plan starts at boundary");
}
//NEUQ_LCD200_DisplayString的作用是在特定坐标的位置写出不会改变的量

void Led_Display_Refresh(uint8_t current, uint8_t pending, uint32_t battery_mv)
{
    /* 固定宽度覆盖旧值，避免数字位数变化后残留。 */
    NEUQ_LCD200_Display_int32(145, 55, (int32_t)(current + 1U), 1);
    NEUQ_LCD200_Display_int32(145, 90, (int32_t)(pending + 1U), 1);
    NEUQ_LCD200_DisplayFloat(145, 125, (float)battery_mv / 1000.0f, 5, 2);
    NEUQ_LCD200_DisplayString(235, 125, "V");
}
//DisplayString：显示固定文字/标签，画一次就行（静态页）
//Display_int32：显示会变的整数，需要反复刷新（刷新页）
//DisplayFloat：显示会变的小数，需要反复刷新（刷新页）

// **************************** 代码区域 ****************************

void Led_Display_Init(void)
{
    NEUQ_LCD200_init();
    App_DrawStaticPage();
}
