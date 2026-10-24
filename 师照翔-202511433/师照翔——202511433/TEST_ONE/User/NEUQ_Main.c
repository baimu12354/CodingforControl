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
#include<stdint.h>
// **************************** 代码区域 ****************************
void User_Main(void)
{
    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();
    //其他初始化
    NEUQ_Timer_init();  
    NEUQ_PIT_init_ms(PIT_CH1, 10);
    NEUQ_Key_Init();
    NEUQ_GPIO_init(PA8, GPIO_Mode_Output_PushPull, 1);

    // 菜单初始化
    Menu_Init();
    NEUQ_Delay_ms(200);

    start_tick = NEUQ_Count_ms();
    while(1)
    {
        // 执行当前LED闪烁策略
        switch(stratage_run) { 
            case 1: LED_Stratage1(); break;
            case 2: LED_Stratage2(); break;
            case 3: LED_Stratage3(); break;
            case 4: LED_Stratage4(); break;
        }

        // 处理按键并更新菜单
        KEY_EVENT_ENUM key = NEUQ_Key_Event();
        if(key != KEY_EVENT_NONE){
            if(Menu_Update(key)){
                Menu_Show();
            }
        }

        //切换方案
        if(is_finish == 1 && stratage_req != stratage_run){
            stratage_run = stratage_req;
            step = 0;
            start_tick = NEUQ_Count_ms();
            is_finish = 0;
            Menu_Show(); // 切换后刷新屏幕
        }
        /*key_test = NEUQ_Key_Event();
        if(key_test == KEY_DOWN_SHORT)
        {
            // 按一次DOWN键松开，LED翻转一次
            NEUQ_GPIO_write(PA8, !NEUQ_GPIO_read(PA8));
        }*/
    }
}