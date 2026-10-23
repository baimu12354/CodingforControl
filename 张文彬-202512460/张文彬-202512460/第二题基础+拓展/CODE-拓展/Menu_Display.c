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

static const char *g_axis_name[6] = {"AX", "AY", "AZ", "GX", "GY", "GZ"};
static AppPage g_page = PAGE_HOME;
static uint8_t g_home_cursor = 0;
static uint8_t g_filter_cursor = 0;
static uint8_t g_stream_enabled = 0;
static uint32_t g_saved_message_until = 0;

static void App_DrawHome(void)
{
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(10, 8, "SECOND TASK - FULL");
    NEUQ_LCD200_DisplayString(35, 55, "Six-axis data");
    NEUQ_LCD200_DisplayString(35, 95, "Wave & filter");
    NEUQ_LCD200_DisplayString(35, 135, "Attitude solve");
    NEUQ_LCD200_DisplayString(10, 205, "UP/DOWN + OK");
}

static void App_DrawSixAxis(void)
{
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(10,5,"SIX-AXIS DATA");
    NEUQ_LCD200_DisplayString(10,35,"AX(g):");
    NEUQ_LCD200_DisplayString(10,65,"AY(g):");
    NEUQ_LCD200_DisplayString(10,95,"AZ(g):");
    NEUQ_LCD200_DisplayString(10,125,"GX(dps):");
    NEUQ_LCD200_DisplayString(10,155,"GY(dps):");
    NEUQ_LCD200_DisplayString(10,185,"GZ(dps):");
    NEUQ_LCD200_DisplayString(235,215,"BACK");
}

static void App_DrawFilter(void)
{
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(10,5,"WAVE & FILTER");
    NEUQ_LCD200_DisplayString(35,45,"Axis:");
    NEUQ_LCD200_DisplayString(35,82,"Stream:");
    NEUQ_LCD200_DisplayString(35,119,"Alpha:");
    NEUQ_LCD200_DisplayString(35,156,"Save to Flash");
    NEUQ_LCD200_DisplayString(10,205,"UP/DOWN, OK, BACK");
}

static void App_DrawAttitude(void)
{
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(10,5,"QUATERNION ATTITUDE");
    NEUQ_LCD200_DisplayString(10,48,"Roll  (deg):");
    NEUQ_LCD200_DisplayString(10,85,"Pitch (deg):");
    NEUQ_LCD200_DisplayString(10,122,"Yaw   (deg):");
    NEUQ_LCD200_DisplayString(10,165,"Period: 10 ms");
    NEUQ_LCD200_DisplayString(10,195,"X:fwd Y:left Z:up");
    NEUQ_LCD200_DisplayString(250,215,"BACK");
}

void Menu_Display_EnterPage(AppPage page)
{
    g_page = page;
    if (page==PAGE_HOME) App_DrawHome();
    else if (page==PAGE_SIX_AXIS) App_DrawSixAxis();
    else if (page==PAGE_FILTER) App_DrawFilter();
    else App_DrawAttitude();
}

void Menu_Display_ProcessKeys(uint8_t keys)
{
    if (g_page == PAGE_HOME)
    {
        if (keys & (1U<<KEY_UP)) g_home_cursor=(uint8_t)((g_home_cursor+2U)%3U);
        if (keys & (1U<<KEY_DOWN)) g_home_cursor=(uint8_t)((g_home_cursor+1U)%3U);
        if (keys & (1U<<KEY_OK)) Menu_Display_EnterPage((AppPage)(g_home_cursor+1U));
    }
    else if (keys & (1U<<KEY_BACK)) Menu_Display_EnterPage(PAGE_HOME);
    else if (g_page == PAGE_FILTER)
    {
        if (keys & (1U<<KEY_UP)) g_filter_cursor=(uint8_t)((g_filter_cursor+3U)%4U);
        if (keys & (1U<<KEY_DOWN)) g_filter_cursor=(uint8_t)((g_filter_cursor+1U)%4U);
        if (keys & (1U<<KEY_OK))
        {
            if (g_filter_cursor==0U) g_config.send_axis=(uint8_t)((g_config.send_axis+1U)%6U);
            else if (g_filter_cursor==1U) g_stream_enabled=(uint8_t)!g_stream_enabled;
            else if (g_filter_cursor==2U)
            {
                g_config.alpha_milli += 50U;
                if (g_config.alpha_milli>950U) g_config.alpha_milli=50U;
            }
            else
            {
                (void)Imu_Settings_Save();
                g_saved_message_until=HAL_GetTick()+1000UL;
            }
        }
    }
}

void Menu_Display_Refresh(void)
{
    uint8_t i;
    if (g_page==PAGE_HOME)
    {
        for (i=0;i<3U;i++)
            NEUQ_LCD200_DisplayString(10,(uint16_t)(55U+40U*i),i==g_home_cursor?">":" ");
        NEUQ_LCD200_DisplayString(230,8,g_mpu_ok?"MPU OK":"MPU ERR");
    }
    else if (g_page==PAGE_SIX_AXIS)
    {
        for (i=0;i<6U;i++)
            NEUQ_LCD200_DisplayFloat(145,(uint16_t)(35U+30U*i),g_imu.raw[i],8,3);
    }
    else if (g_page==PAGE_FILTER)
    {
        for (i=0;i<4U;i++)
            NEUQ_LCD200_DisplayString(10,(uint16_t)(45U+37U*i),i==g_filter_cursor?">":" ");
        NEUQ_LCD200_DisplayString(155,45,(char *)g_axis_name[g_config.send_axis]);
        NEUQ_LCD200_DisplayString(155,82,g_stream_enabled?"ON ":"OFF");
        NEUQ_LCD200_DisplayFloat(155,119,(float)g_config.alpha_milli/1000.0f,4,2);
        NEUQ_LCD200_DisplayString(210,156,
            (int32_t)(g_saved_message_until-HAL_GetTick())>0?"SAVED":"     ");
    }
    else
    {
        NEUQ_LCD200_DisplayFloat(185,48,g_attitude.roll,8,2);
        NEUQ_LCD200_DisplayFloat(185,85,g_attitude.pitch,8,2);
        NEUQ_LCD200_DisplayFloat(185,122,g_attitude.yaw,8,2);
    }
}

// **************************** 代码区域 ****************************

uint8_t Menu_Display_StreamEnabled(void)
{
    return g_stream_enabled;
}
