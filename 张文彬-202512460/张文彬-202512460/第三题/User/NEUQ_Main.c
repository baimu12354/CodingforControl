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
* 组织名称          东北大学秦皇岛分校智能车实验室
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

void User_Main(void)
{
    uint32_t last_vofa_ms = 0UL;
    uint32_t now_ms;

    NEUQ_Delay_init();

    /* 上电先初始化为停机，再打开命令串口，最后启动 10 ms 控制中断。 */
    Motor_Control_Init();
    Motor_Protocol_Init();
    NEUQ_PIT_init_ms(MOTOR_CONTROL_PIT, MOTOR_CONTROL_PERIOD_MS);

    while (1)
    {
        /* 主循环只负责通信；电机闭环始终由 PIT 中断独立运行。 */
        Motor_Protocol_ProcessRx();//此处在主循环里面解析命令然后再执行命令

        now_ms = HAL_GetTick();
        if (Motor_Protocol_StatusPending() ||
            (uint32_t)(now_ms - last_vofa_ms) >= MOTOR_VOFA_PERIOD_MS)
        {
            last_vofa_ms = now_ms;
            Motor_Protocol_SendVofaStatus();
        }
    }
}

uint8_t a;
//在该项目只要重点的看这三个函数即可
//NEUQ_Uart_write_data()
//负责：把已经组装好的字节逐个送上串口

//Protocol_WriteFloatLE()
//负责：一个数怎么变成4字节

//Motor_Protocol_SendVofaStatus()
//负责：11个通道怎么排列、在哪里加帧尾
