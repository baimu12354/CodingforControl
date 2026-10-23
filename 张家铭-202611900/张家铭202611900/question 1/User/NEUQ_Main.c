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
volatile uint16_t tim_10ms_cnt;
    volatile uint8_t now_scheme = 1;
    volatile uint8_t next_scheme = 1;

   void NEUQ_PIT_2_CALLBACK(void){tim_10ms_cnt++;

        if (tim_10ms_cnt >=200){
        tim_10ms_cnt =0;
       now_scheme=next_scheme;
    }
    switch(now_scheme){
   case 1:if (tim_10ms_cnt<100){NEUQ_GPIO_write(PA8, 1);}
   else{NEUQ_GPIO_write(PA8, 0);}
   break;
   case 2:if(tim_10ms_cnt<20){NEUQ_GPIO_write(PA8, 1);}
    else{NEUQ_GPIO_write(PA8, 0);}
   break;
    case 3:if(tim_10ms_cnt<20||tim_10ms_cnt>=40&&tim_10ms_cnt<60){NEUQ_GPIO_write(PA8, 1);}
    else{NEUQ_GPIO_write(PA8, 0);}
   break;
    case 4:if(tim_10ms_cnt<20||tim_10ms_cnt>=40&&tim_10ms_cnt<100){NEUQ_GPIO_write(PA8, 1);}
    else{NEUQ_GPIO_write(PA8, 0);}
   break;
      }}
void User_Main(void){

    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();
NEUQ_Key_Init();
NEUQ_LCD200_init();
NEUQ_GPIO_init(PA8, GPIO_Mode_Output_PushPull, GPIO_High);
NEUQ_GPIO_write(PA8, 0);
NEUQ_PIT_init_ms(PIT_CH1,10);
NEUQ_PIT_init_ms(PIT_CH2,10);

NEUQ_LCD200_SetPenColor(LCD_BLACK);
NEUQ_LCD200_SetBackColor(LCD_WHITE);
NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
NEUQ_LCD200_Clear();
NEUQ_LCD200_DisplayString(0, 0, "NOW :");
NEUQ_LCD200_DisplayString(0, 32, "NEXT:");

KEY_EVENT_ENUM temp=KEY_EVENT_NONE;
static uint8_t last_now=0, last_next=0;+
while(1){temp=NEUQ_Key_EventGet();
    if(temp==KEY_UP_SHORT){
        next_scheme++;
        if(next_scheme>4) next_scheme=1;
    }
    if(now_scheme!=last_now){
        NEUQ_LCD200_Display_int32(64,0,now_scheme,1);
        last_now=now_scheme;
    }
    if(next_scheme!=last_next){
        NEUQ_LCD200_Display_int32(64,32,next_scheme,1);
        last_next=next_scheme;
    }
  }
}