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
    volatile uint8_t LEDflag=1,PITflag=0,LEDmode=0,frequenCount=0,stap=0;
    volatile uint32_t timRecord=0;
void User_Main(void){
    uint8_t mod;
  
    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();

    NEUQ_Timer_init();
    NEUQ_PIT_init_ms(PIT_CH1,200);
    NEUQ_EXTI_Init(PE9, EXTI_Edge_Down);
    NEUQ_GPIO_init(PA8, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_GPIO_init(PE9, GPIO_Mode_Input_PullUp, 0);
    adc_init( ADC1_CH3_PA3, ADC_12BIT);

//NEUQ_GPIO_write(PA8,1);

    NEUQ_LCD200_init();
    //NEUQ_LCD200_SetDirection(LCD200_DIR_ENUM dir);
    NEUQ_LCD200_SetPenColor(LCD_BLACK); 
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_DisplayChar(10,10+(LEDmode)*40,'>');
    NEUQ_LCD200_DisplayString(30,10,"MOD1,1000/1000");
    NEUQ_LCD200_DisplayString(30,50,"MOD2,200/1800");
    NEUQ_LCD200_DisplayString(30,90,"MOD3,200/200/200/1400");
    NEUQ_LCD200_DisplayString(30,130,"MOD4,200/200/600/1000");

    while(1){
        // NEUQ_GPIO_toggle(PA8);
        // NEUQ_Delay_ms(500);

        // if(NEUQ_GPIO_read(PE9)==0)
        // {
        //          NEUQ_GPIO_write(PA8,1);
        // }

        if (LEDflag==1)//按键消抖
        {
            if(NEUQ_Get_Timer_Value_ms()-timRecord>=20)
            {
                if(NEUQ_GPIO_read(PE9)==0)
                {
                    LEDmode++;
                    if(LEDmode>3)
                    {
                        LEDmode=0;
                    }
                    NEUQ_LCD200_ClearRect(10,10,20,150);
                    NEUQ_LCD200_DisplayChar(10,10+(LEDmode)*40,'>');
                    // NEUQ_LCD200_SetPenColor(LCD_BLACK); 
                    // NEUQ_LCD200_SetBackColor(LCD_WHITE);
                }
                LEDflag=0;

            }
            /* code */
        }

    if (PITflag==1)
    {
        // NEUQ_LCD200_Display_int32(10,180,LEDmode*1000+mod*100+ PITflag*10+stap,10);
        //NEUQ_LCD200_DisplayString(10,180,"adc:");
        //NEUQ_LCD200_DisplayFloat(50,180,(float)adc_convert(ADC1_CH3_PA3)*3.3f/4095.f,6,2);
        if(frequenCount>9)
     {
        mod=LEDmode;
        frequenCount=0;
        stap=0;
     }
    
        switch(mod){
            case 0:
                if(stap==0)
                NEUQ_GPIO_write(PA8,GPIO_High);
                if(stap==5)
                NEUQ_GPIO_write(PA8,GPIO_Low);
                break;
            case 1:
                if(stap==0) NEUQ_GPIO_write(PA8,GPIO_High);
                if(stap==1)NEUQ_GPIO_write(PA8,GPIO_Low);
                break;
            case 2:
                if(stap==0)NEUQ_GPIO_write(PA8,GPIO_High);
                if(stap==1)NEUQ_GPIO_write(PA8,GPIO_Low);
                if(stap==2)NEUQ_GPIO_write(PA8,GPIO_High);
                if(stap==3)NEUQ_GPIO_write(PA8,GPIO_Low);
                break;
            case 3:
                if(stap==0)NEUQ_GPIO_write(PA8,GPIO_High);
                if(stap==1)NEUQ_GPIO_write(PA8,GPIO_Low);     
                if(stap==2)NEUQ_GPIO_write(PA8,GPIO_High);
                if(stap==5)NEUQ_GPIO_write(PA8,GPIO_Low);  
                break;
        }
        stap++;
     
     frequenCount++;
     PITflag=0;
        /* code */
    }
    


    }
}
