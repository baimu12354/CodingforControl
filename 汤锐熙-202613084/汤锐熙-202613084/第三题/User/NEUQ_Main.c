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
#include "../CODE/uart_packet.h"
#include "../CODE/NEUQ_Motor.h"

// **************************** 代码区域 ****************************
void User_Main(void){
    // 初始化默认调试串口（UART_0 = USART1，9600）
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();




    NEUQ_PIT_init_ms(PIT_CH1,1);  //定时向上位机发送状态50ms, 控制电机20ms, 读取编码器10ms
    NEUQ_Motor_Init();
    // 使能 UART0 接收中断（接收解包模块初始化）
    UartPacket_Init();


    while(1){
        UartPacket_Task();   // 解包收到的字节，更新 run_flag/target_rpm/kp/ki/kd
        // 之后在这里用这些全局变量做闭环控制、回传、显示等
        if (send_flag){
            send_flag = 0;              // 先清标志，再做耗时的发送
            UartPacket_VofaSend();        // VOFA JustFloat：发 5 个 float 给上位机画图
        }
    }
}
