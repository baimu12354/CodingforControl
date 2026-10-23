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

    NEUQ_LCD200_init();

    NEUQ_GPIO_init(PB0,GPIO_Mode_Output_PushPull,1);



    NEUQ_PIT_init_ms(PIT_CH1,10); 
    Cmd_Inite();
    NEUQ_LCD200_DisplayString(10,200,"ready");
// NEUQ_Encoder_init(ENCODER_TIMER,ENCODER_PHRASE_A_PIN,ENCODER_PHRASE_B_PIN);
    Motor_Inite();
// Motor_SetDuty(-1000);//用于测试电机口是不是接对了
// NEUQ_PWM_set_duty(MOTOR_PWM_PIN,500);

//测试数据
// Motor_Data.Kp=2;
// Motor_Data.Ki=0.01;
// Motor_Data.Kd=0;
// Motor_Data.run=1;
// Motor_Data.target=5000;
uint8_t send_byte=0;
    while(1)
    {
    
    // NEUQ_LCD200_Clear();
    Cmd_UnpackData();

    //在屏幕上显示数据用于调试
//     NEUQ_LCD200_Display_int32(100,10,Motor_Data.actual,8);
//     NEUQ_LCD200_Display_int32(100,30,Motor_Data.target,8);
//     NEUQ_LCD200_DisplayFloat(100,50,Motor_Data.error,8,4);
//     NEUQ_LCD200_DisplayFloat(100,70,Motor_Data.last_error,8,4);
//     NEUQ_LCD200_DisplayFloat(100,90,Motor_Data.last_last_error,8,4);
//     NEUQ_LCD200_Display_int32(100,110,Motor_Data.PWM_Record,8);NEUQ_LCD200_Display_int32(150,110,Motor_Data.output,8);
//     NEUQ_LCD200_DisplayFloat(100,130,Motor_Data.integral,8,4);
//     NEUQ_LCD200_DisplayFloat(150,190,Motor_Data.Kd,8,4);
//     NEUQ_LCD200_DisplayFloat(100,170,Motor_Data.Ki,8,4);
//     NEUQ_LCD200_DisplayFloat(100,150,Motor_Data.Kp,8,4);


//    NEUQ_LCD200_Display_int32(100,200,Motor_Data.run,8);


//发送数据字节包，没用上，可能还有残留的文件没被注释
    // if(NEUQ_FIFO_read_byte(&send_fifo,&send_byte))
    // {
    //     NEUQ_Uart_write_byte(UART_0,send_byte);

    // }


    
//  Motor_SetDuty(Motor_Data.PWM_Record);//test

    // Motor_Data.last_last_error=Motor_Data.last_error;
    // Motor_Data.last_error=Motor_Data.error;
    // Motor_Data.error=Motor_Data.target-Motor_Data.actual;
    // Motor_Data.integral+=Motor_Data.error;
    // if(Motor_Data.integral>2000)Motor_Data.integral=2000;//?
    // if(Motor_Data.integral<-2000)Motor_Data.integral=-2000;
    // Motor_Data.output=Motor_Data.Kp*(Motor_Data.error-Motor_Data.last_error)
    //     +Motor_Data.Ki*Motor_Data.integral
    //     +Motor_Data.Kd*(Motor_Data.error-2*Motor_Data.last_error+Motor_Data.last_last_error);
    // Motor_Data.PWM_Record+=Motor_Data.output;
    // if (Motor_Data.PWM_Record>10000)Motor_Data.PWM_Record=10000;
    // if (Motor_Data.PWM_Record<-10000)Motor_Data.PWM_Record=-10000;
    // if(Motor_Data.run==1)
    // {
    //     Motor_SetDuty(Motor_Data.PWM_Record);
    // }
    // else 
    // {
    //     Motor_SetDuty(0);
    // }
    

    // printf("%d\n",NEUQ_Encoder_Read(ENCODER_TIMER));
    // NEUQ_Delay_ms(10);
    // NEUQ_LCD200_Display_int32(10,10,NEUQ_Encoder_Read(ENCODER_TIMER),8);
    // Motor_Data.actual=NEUQ_Encoder_Read(ENCODER_TIMER)*250;
    
    // NEUQ_Encoder_Clear(ENCODER_TIMER);
    
    }
}