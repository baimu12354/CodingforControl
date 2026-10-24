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
  
    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();

    // 1. 电机：PWM + 方向引脚
    Motor_Init();

    // 2. 测速与闭环控制：编码器 + 默认 PID 参数，上电处于停止状态
    Speed_Init();

    // 3. 串口协议：UART0 @115200，开接收中断，等待上位机字节包
    Protocol_Init();

    // 4. 10ms 定周期中断：测速 + PID + PWM 更新（高精度控制放中断）
    NEUQ_PIT_init_ms(PIT_CH1, 10);
    

    while(1){
        // 主循环：解析串口字节包（分次到达/连续多帧都能处理）
        Protocol_Task();
        static uint16_t cnt = 0;
        if (++cnt >= 10) {
            cnt = 0; 
        }

        // 主循环：VOFA JustFloat 回传 目标/实际转速 + PID 参数
        Protocol_Send_Status();
    }
}