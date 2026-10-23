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
typedef struct
{
    uint16_t duration_ms;
    GPIO_Level led_level;
} LedStep;

/* 四套动作均严格组成 2000 ms。 */
static const LedStep g_led_steps[4][4] =
{
    {{1000, GPIO_High}, {1000, GPIO_Low }, {0, GPIO_Low},  {0, GPIO_Low}},
    {{ 200, GPIO_High}, {1800, GPIO_Low }, {0, GPIO_Low},  {0, GPIO_Low}},
    {{ 200, GPIO_High}, { 200, GPIO_Low }, {200, GPIO_High},{1400, GPIO_Low}},
    {{ 200, GPIO_High}, { 200, GPIO_Low }, {600, GPIO_High},{1000, GPIO_Low}}
};

static const uint8_t g_led_step_count[4] = {2, 2, 4, 4};



void Led_Control_Update(uint8_t scheme, uint32_t elapsed_ms)
{
    uint32_t boundary = 0;
    uint8_t i;

    for (i = 0; i < g_led_step_count[scheme]; i++)
    {
        boundary += g_led_steps[scheme][i].duration_ms;
        if (elapsed_ms < boundary)
        {
            NEUQ_GPIO_write(APP_LED_PIN, g_led_steps[scheme][i].led_level);
            return;
        }
    }
    NEUQ_GPIO_write(APP_LED_PIN, GPIO_Low);//目的是防止计数的时间超过了一定的值导致了结果的混乱
}

void Led_Control_Init(void)
{
    NEUQ_GPIO_init(APP_LED_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
}
