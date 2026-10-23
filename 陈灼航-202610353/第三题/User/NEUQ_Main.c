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

// **************************** 可调参数 ****************************
#define MOTOR_KP             1500.0f   // 比例系数
#define MOTOR_KI             50.0f     // 积分系数
#define MOTOR_KD             0.0f      // 微分系数
// *****************************************************************

// **************************** 代码区域 ****************************
void User_Main(void){
    NEUQ_Debug_init();      // 初始化调试串口 UART_0（115200）
    // 延迟初始化函数
    NEUQ_Delay_init();      // DWT 延时初始化

    // ---- 电机控制相关 ----
    NEUQ_MOTOR_INIT();      // 初始化 PWM(TIM2_CH3 / PA2)、方向脚(PA5)、PID。
    NEUQ_GPIO_init(PB0, GPIO_Mode_Output_PushPull, GPIO_High);   // PB0 是板载 Flash 的片选脚：上电拉高，防止 Flash 被误选
    NEUQ_Encoder_init(Encoder_TIM4, Encoder_TIM4_PhaseA_PB6, Encoder_TIM4_PhaseB_PB7);   // 编码器：TIM4 正交解码，A 相 PB6、B 相 PB7

    // ---- 通信相关 ----
    NEUQ_VOFA_INIT();       // 初始化接收 FIFO + 打开串口接收中断（在中断回调里接 VOFA）
    NEUQ_MOTOR_SetPID(MOTOR_KP, MOTOR_KI, MOTOR_KD);   // 用上面的宏给 PID 上电默认参数

    // ---- 控制节拍：10ms 周期中断（PIT_CH1 = TIM6）----
    NEUQ_PIT_init_ms(PIT_CH1,10);   // 10ms
    while(1){
        NEUQ_VOFA_Process();        // 解析上位机下发的指令
        NEUQ_VOFA_Send();           // 20ms
    }
}
