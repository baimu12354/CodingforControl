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
#include "../CODE/NEUQ_Config.h"      // 第三题配置（含 NEUQ_PIT_CHANNEL_INDEX / NEUQ_COMM_UART_INDEX）
#include "../CODE/NEUQ_Control.h"     // NEUQ_Control_ISR()
#include "../CODE/NEUQ_Comm.h"        // NEUQ_Comm_RxByte()

// **************************** PIT中断函数 ****************************
// 第三题：定周期控制节拍（读编码器 -> PID -> 更新 PWM），周期 = NEUQ_CONTROL_PERIOD_MS
void NEUQ_PIT_1_CALLBACK(void){
#if (NEUQ_PIT_CHANNEL_INDEX == 1)
    NEUQ_Control_ISR();
#endif
}
void NEUQ_PIT_2_CALLBACK(void){
#if (NEUQ_PIT_CHANNEL_INDEX == 2)
    NEUQ_Control_ISR();
#endif
}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 第三题：上位机字节包接收，中断里只把字节压进环形缓冲，解析在主循环做
void TS_UART_0_RX_CALLBACK(uint8_t dat){
#if (NEUQ_COMM_UART_INDEX == 0)
    NEUQ_Comm_RxByte(dat);
#endif
}

void TS_UART_1_RX_CALLBACK(uint8_t dat){
#if (NEUQ_COMM_UART_INDEX == 1)
    NEUQ_Comm_RxByte(dat);
#endif
}

void TS_UART_2_RX_CALLBACK(uint8_t dat){
#if (NEUQ_COMM_UART_INDEX == 2)
    NEUQ_Comm_RxByte(dat);
#endif
}

void TS_UART_3_RX_CALLBACK(uint8_t dat){
#if (NEUQ_COMM_UART_INDEX == 3)
    NEUQ_Comm_RxByte(dat);
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * ★ 串口错误回调（必须实现，否则拖滑条时串口会"假死"）
 *
 *   为什么必须有：库用 HAL_UART_Receive_IT() 逐字节接收。一旦总线上出现
 *   溢出(ORE)/帧错(FE)/噪声(NE)/校验错(PE) 中的任何一种，HAL 会
 *   · 停掉本次接收（UART_EndRxTransfer）
 *   · 调用 HAL_UART_ErrorCallback
 *   如果这里不重新 HAL_UART_Receive_IT()，接收就再也不会被重新挂起 ——
 *   表现为：**发命令完全没反应，必须复位单片机才恢复**。
 *
 *   触发场景正是 VOFA+ 拖动滑条：短时间内连续下发大量帧，主循环又在
 *   阻塞式发送回传帧，很容易出现一次 ORE。
 *------------------------------------------------------------------------------------------------------------------*/
extern uint8_t UART_0_RX_dat;                 /* 库里定义，接收中断的 1 字节缓冲 */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL) { return; }

    if (huart->Instance == USART1)
    {
        /* ① 清掉所有错误标志，否则会立刻再次进入错误回调 */
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        /* ② 重新挂起单字节接收中断 —— 这一步是关键，漏了串口就永久收不到数据 */
        HAL_UART_Receive_IT(huart, &UART_0_RX_dat, 1);
    }
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