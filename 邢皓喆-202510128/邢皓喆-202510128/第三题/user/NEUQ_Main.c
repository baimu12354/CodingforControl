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

// **************************** 代码区域 ****************************
void User_Main(void){

    NEUQ_Debug_init();
    NEUQ_Delay_init();
    NEUQ_Encoder_init(Encoder_TIM3,Encoder_TIM3_PhaseA_PA6, Encoder_TIM3_PhaseB_PA7);
    NEUQ_GPIO_init(PC7,GPIO_Mode_Output_PushPull, GPIO_High);
    NEUQ_PWM_init(TIM8_CH1_PC6, 1000, 0);
    NEUQ_FIFO_init(&rx_fifo, rx_buffer, sizeof(rx_buffer));
    NEUQ_Uart_init(UART_0, UART_BAUD_115200);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 5);
    Ipid_parameters_setting(&speed_pid, 0.0f, 0.0f, 0.0f);
    NEUQ_PIT_init_ms(PIT_CH1, 10);
    while (1)
    {
        uint8_t dat;

        while (NEUQ_FIFO_read_byte(&rx_fifo, &dat) == NEUQ_FIFO_OK)
        {
            cmd_control_init(dat);
        }
        uint32_t now = TIMER_GET_MS();
        if ((uint32_t)(now - last_send) >= 20U)
        {
            last_send = now;
            VOFA_SENT();
        }
    }
}