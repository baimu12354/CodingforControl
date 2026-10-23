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


extern void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

// **************************** 代码区域 ****************************
void User_Main(void){

    // 延迟初始化函数
    NEUQ_Delay_init();

    // 初始化数据吞吐（FIFO + 协议状态机）
    NEUQ_Pack_init();
    // 协议串口
    NEUQ_Uart_init(UART_0, UART_BAUD_115200);
    // 打开接收中断
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 5);

    // 初始化pwm
    NEUQ_PWM_init(TIM2_CH3_PA2,1000,0);

    // 方向引脚 PA5（电机驱动 DIR）：PA2 只出 PWM 占空比，转向由 PA5 决定
    NEUQ_GPIO_init(PA5, GPIO_Mode_Output_PushPull, GPIO_Low);

    // 初始化编码器
    NEUQ_Encoder_init(Encoder_TIM4, Encoder_TIM4_PhaseA_PB6, Encoder_TIM4_PhaseB_PB7);

    // 测速/控制周期 10ms
    NEUQ_PIT_init_ms(PIT_CH1,10);

    // 上电默认 PID（rpm：Kp=3/Ki=0.15/Kd=0，）
    pid_parameters_setting(&motor_pid,3.0f,0.15f,0.0f);

    

    while(1){
        uint8_t b;
        while(NEUQ_FIFO_read_byte(&uart_rx_fifo, &b) == NEUQ_FIFO_OK){
            protocol_parse_byte(b);
        }

        // 每个测速周期（10ms）回传一次
        if(PIT1_flag == 1){
            PIT1_flag = 0;
            protocol_send_echo();
        }
    }
}
