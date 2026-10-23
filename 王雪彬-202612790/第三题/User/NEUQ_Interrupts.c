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
* 文件名称          NEUQ_Interrupts
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
* 2025-09-27        张瑞        加入外部中断回调
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

#define FF_GAIN  8.0f   // 前馈增益（占空比/rpm）

// **************************** PIT中断函数 ****************************
volatile uint8_t PIT1_flag = 0;


void NEUQ_PIT_1_CALLBACK(void){
    static int32_t last_count = 0;
    static float  filtered_speed = 0.0f;   // 一阶低通滤波状态：平滑低分辨率编码器的量化阶跃
    int32_t now = NEUQ_Encoder_Read(Encoder_TIM4);
    int32_t delta = now - last_count;   // 10ms 内的编码器计数
    last_count = now;

    
    if(delta >  32767) delta -= 65536;
    if(delta < -32767) delta += 65536;

    
    float raw_speed = -(float)delta * 13.636f;
    filtered_speed += 0.2f * (raw_speed - filtered_speed);
    actual_speed = (int32_t)filtered_speed;

    if(motor_enable){
       
        float max_I = (motor_pid.Ki > 0.001f) ? (2000.0f / motor_pid.Ki) : 0.0f;
        float out = Unclipped_Positional_PID(&motor_pid,
                                            (float)target_speed,
                                            (float)actual_speed,
                                            max_I);
        out += FF_GAIN * (float)target_speed;   // 前馈：起步瞬间给到接近稳态占空比，PID 只做小纠偏

        
        if(out > 10000.0f){
            if(motor_pid.Ki > 0.001f) motor_pid.Ek_total -= (out - 10000.0f) / motor_pid.Ki;
            out = 10000.0f;
        }else if(out < -10000.0f){
            if(motor_pid.Ki > 0.001f) motor_pid.Ek_total -= (out + 10000.0f) / motor_pid.Ki;
            out = -10000.0f;
        }
        
        // 方向：out>=0 → PA5 高（正转）；out<0 → PA5 低（反转）
        if(out >= 0.0f){
            NEUQ_GPIO_write(PA5, GPIO_High);
        }else{
            NEUQ_GPIO_write(PA5, GPIO_Low);
            out = -out;
        }
        if(out > 10000.0f)  out = 10000.0f;

        NEUQ_PWM_set_duty(TIM2_CH3_PA2,(uint32_t)out);
    }else{
        NEUQ_PWM_set_duty(TIM2_CH3_PA2,0);
        NEUQ_GPIO_write(PA5, GPIO_Low);   // 停止：方向释放为低
    }



    PIT1_flag = 1;
}
void NEUQ_PIT_2_CALLBACK(void){

}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
void TS_UART_0_RX_CALLBACK(uint8_t dat){
    NEUQ_FIFO_write_byte(&uart_rx_fifo, dat);   // 每收到 1 字节，丢进 FIFO
}

void TS_UART_1_RX_CALLBACK(uint8_t dat){
    
}

void TS_UART_2_RX_CALLBACK(uint8_t dat){
    
}

void TS_UART_3_RX_CALLBACK(uint8_t dat){
    
}
// **************************** 串口中断函数 ****************************


// **************************** 外部中断函数 ****************************
void NEUQ_EXTI0_Callback(void)  {

}

void NEUQ_EXTI1_Callback(void)  {

}

void NEUQ_EXTI2_Callback(void)  {

}

void NEUQ_EXTI3_Callback(void)  {

}

void NEUQ_EXTI4_Callback(void)  {

}

void NEUQ_EXTI5_Callback(void)  {

}

void NEUQ_EXTI6_Callback(void)  {

}

void NEUQ_EXTI7_Callback(void)  {

}

void NEUQ_EXTI8_Callback(void)  {

}

void NEUQ_EXTI9_Callback(void)  {

}

void NEUQ_EXTI10_Callback(void) {

}

void NEUQ_EXTI11_Callback(void) {

}

void NEUQ_EXTI12_Callback(void) {

}

void NEUQ_EXTI13_Callback(void) {

}

void NEUQ_EXTI14_Callback(void) {

}

void NEUQ_EXTI15_Callback(void) {

}
// **************************** 外部中断函数 ****************************