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
#define DEMO_KEY_PIN PA0
#define DEMO_LED_PIN PA8
uint32_t tim_cnt = 0;                  //PIT节拍计数器，1次代表10ms 全局变量
typedef struct{
    uint8_t  run_mode;                 //当前执行方案
    uint8_t  next_mode;                //待执行方案
    uint8_t  old_run;                  //LCD缓存：上一次显示的当前方案
	uint8_t  old_next;                 //LCD缓存：上一次显示的待执行方案
} LED_ModeTypedef;
LED_ModeTypedef led_ctrl = {1,1,0,0};
// **************************** 代码区域 ****************************
void User_Main(void){
   
    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();
    NEUQ_Key_Init();
    NEUQ_GPIO_init(DEMO_LED_PIN, GPIO_Mode_Output_PushPull, GPIO_High);
    NEUQ_PIT_init_ms(PIT_CH1,10);
    NEUQ_LCD200_init();
    while(1){
        NEUQ_Key_Scan();
        uint8_t key_event = NEUQ_Key_EventGet();
        if(key_event == 1){
            led_ctrl.next_mode ++;
            if(led_ctrl.next_mode > 4){
                led_ctrl.next_mode = 1;
            }
        }
         if( tim_cnt >= 200 ){
        tim_cnt = 0;
        led_ctrl.run_mode = led_ctrl.next_mode;
        }
        switch(led_ctrl.run_mode){
            case 1:
            NEUQ_GPIO_write(DEMO_LED_PIN, (tim_cnt < 100) ? GPIO_High : GPIO_Low);
                break;
            case 2:
                NEUQ_GPIO_write(DEMO_LED_PIN, (tim_cnt < 20) ? GPIO_High : GPIO_Low);
                break;
            case 3:
                if(tim_cnt < 20)
                    NEUQ_GPIO_write(DEMO_LED_PIN, GPIO_High);
                else if(tim_cnt < 40)
                    NEUQ_GPIO_write(DEMO_LED_PIN, GPIO_Low);
                else if(tim_cnt < 60)
                    NEUQ_GPIO_write(DEMO_LED_PIN, GPIO_High);
                else
                    NEUQ_GPIO_write(DEMO_LED_PIN, GPIO_Low);
                break;
            case 4:
                if(tim_cnt < 20)
                    NEUQ_GPIO_write(DEMO_LED_PIN, GPIO_High);
                else if(tim_cnt < 40)
                    NEUQ_GPIO_write(DEMO_LED_PIN, GPIO_Low);
                else if(tim_cnt < 100)
                    NEUQ_GPIO_write(DEMO_LED_PIN, GPIO_High);
                else
                    NEUQ_GPIO_write(DEMO_LED_PIN, GPIO_Low);
                break;
            default:
                break;    
       
}
        if(led_ctrl.run_mode != led_ctrl.old_run || led_ctrl.next_mode != led_ctrl.old_next){
            NEUQ_LCD200_Clear();
            NEUQ_LCD200_DisplayString(0, 0, "Mode:");
            NEUQ_LCD200_Display_int32(72, 0, led_ctrl.run_mode, 1);

            NEUQ_LCD200_DisplayString(0, 32, "Next:");
            NEUQ_LCD200_Display_int32(72, 32, led_ctrl.next_mode, 1);

            led_ctrl.old_run = led_ctrl.run_mode;
            led_ctrl.old_next = led_ctrl.next_mode;
        }
    }
}