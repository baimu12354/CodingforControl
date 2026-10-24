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
#include "NEUQ_Display_Refresh.h"

/* ============ LED 时序任务内部状态（仅在 1ms 中断中访问） ============ */
static uint16_t s_phase = 0;                            // 周期内相位 0~1999

// 返回 1 = 该方案该相位下 LED 点亮，0 = 熄灭
static uint8_t LED_On(uint8_t scheme, uint16_t phase){
    switch(scheme){
        case 1: return (phase < 1000);                                 // 亮1000 灭1000
        case 2: return (phase < 200);                                  // 亮200  灭1800
        case 3: return (phase < 200) || (phase >= 400 && phase < 600); // 亮200 灭200 亮200 灭1400
        case 4: return (phase < 200) || (phase >= 400 && phase < 1000);// 亮200 灭200 亮600 灭1000
        default: return 0;
    }
}

// **************************** PIT中断函数 ****************************
void NEUQ_PIT_1_CALLBACK(void){
    // 周期边界：每 2s 交接一次，next 保持不变
    s_phase++;
    if(s_phase >= CYCLE_MS){
        s_phase       = 0;
        g_led_current = g_led_next;   // 待执行方案生效
        g_led_dirty   = 1;
    }

    // 点灯：只依赖 current 和 phase，屏幕刷新 / 按键绝不干扰
    if(LED_On(g_led_current, s_phase)){
        NEUQ_GPIO_write(LED_PIN, GPIO_High);   // 高电平亮
    }else{
        NEUQ_GPIO_write(LED_PIN, GPIO_Low);
    }
}
void NEUQ_PIT_2_CALLBACK(void){

}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
void TS_UART_0_RX_CALLBACK(uint8_t dat){

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