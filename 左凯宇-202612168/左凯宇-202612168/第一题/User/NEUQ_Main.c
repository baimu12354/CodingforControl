
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

volatile uint32_t pit_tick = 0;  // 中断写入，主循环读取。
volatile uint32_t led_tick = 0;
#define LED1 PA8


void User_Main(void){
  
    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();
    //按键初始化
    NEUQ_Key_Init();
    //菜单&屏幕初始化
    NEUQ_Menu_Init();
    //GPIO初始化
    NEUQ_GPIO_init(LED1, GPIO_Mode_Output_PushPull, GPIO_Low);
    //TIM6
    NEUQ_PIT_init_ms(PIT_CH1,10);

    uint32_t led_t = 0;

    while(1){
        uint32_t t = led_tick % 200;

        //20ms刷新菜单
        if ((uint32_t)(pit_tick) >= 2U){
            NEUQ_Menu_Task();
            pit_tick = 0;
        }


    uint32_t now = led_tick;
    //判断自方案开始是否已过2s，以在周期边界生效
    if (now - led_t >= 200U){
        params.action = params.pre;
        led_t += 200U;
    }


        switch (params.action)
        {
            case 1:
                if (t < 100){
                    NEUQ_GPIO_write(LED1,GPIO_High);
                }
                else{
                    NEUQ_GPIO_write(LED1,GPIO_Low);
                }
                break;
            case 2:
                if (t < 20){
                    NEUQ_GPIO_write(LED1,GPIO_High);
                }
                else{
                    NEUQ_GPIO_write(LED1,GPIO_Low);
                }
                break;
            case 3:
                //如果t<20或>=40且<60
                if (t < 20 || (t >= 40 && t < 60)){
                    NEUQ_GPIO_write(LED1,GPIO_High);
                }
                else{
                    NEUQ_GPIO_write(LED1,GPIO_Low);
                }
                break;
            case 4:
                //如果t<20或>=40且<100
                if (t < 20 || (t >= 40 && t < 100)){
                    NEUQ_GPIO_write(LED1,GPIO_High);
                }
                else{
                    NEUQ_GPIO_write(LED1,GPIO_Low);
                }
                break;
            default:
                break;
        }     
    }
}