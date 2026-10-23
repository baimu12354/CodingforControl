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

    Key_Init();
    NEUQ_MPU6050_init();
    Menu_Init();
    NEUQ_Uart_init(UART_0,UART_BAUD_115200);

    NEUQ_Flash_init();
    Config_Load();
    //陀螺仪零偏校准
    Gyro_Bias_Calibration();
    //滤波初始化
    for(uint8_t i = 0; i < 6; i++){
        kalman_init(&kf[i], 0.0f, 5.0f); 
    }

    //应用Flash中的滤波参数
    Filter_ApplyParams(Kalman_q_cnt, Kalman_r_cnt);
            
    NEUQ_PIT_init_ms(PIT_CH1,10);   //每10ms触发一次中断

    

    
    while(1){
        Menu_Task();
        //发送波形
        if(Menu_Getvalue(MENU_Transmission_switch) == 1){
            int32_t sel = (int32_t)Menu_Getvalue(MENU_Select_transmission);
            if(sel < 0) sel = 0;
            if(sel > 5) sel = 5;
            printf("%.2f,%.2f\n",raw[sel], filtered[sel]);
    }
    }
}