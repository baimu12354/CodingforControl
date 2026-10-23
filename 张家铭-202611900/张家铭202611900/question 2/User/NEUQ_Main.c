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

void User_Main(void){
    NEUQ_Debug_init();
    NEUQ_Delay_init();
    NEUQ_Timer_init();          // 启动 TIM13，供 NEUQ_Get_Timer_Value_ms() 使用（菜单刷新/发送节拍依赖它）
    NEUQ_Key_Init();
    NEUQ_PIT_init_ms(PIT_CH1,10);   // 定时扫描按键（NEUQ_Key_Scan 在 PIT1 中断里跑）
    NEUQ_Sensor_Init();
    NEUQ_Flash_init();          // 板载 W25Q16 初始化，必须在姿态零偏加载之前
    NEUQ_Attitude_Init();    // 加载已保存零偏；无有效数据时现场标定并保存
    NEUQ_Menu_Init();

    while(1){
        NEUQ_Sensor_Task();    // 一直采集+滤波，切页面不受影响
        NEUQ_Attitude_Task();  // Mahony 姿态更新
        NEUQ_Menu_Task();      // 按键 + 菜单显示
    }
}

// Q2 用不到 PIT2，但库在 TIM7 中断里会调它（非 weak），必须留个空实现，否则链接报错
void NEUQ_PIT_2_CALLBACK(void){}
