/*********************************************************************************************************************
* 文件名称          NEUQ_MotorHw
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第三题：PWM/方向输出 + TIM4 硬件编码器 + TIM5 1 ms 节拍 + USART1 非阻塞收发
*
* 接线：
*   PWM       PA2  TIM2_CH3   → 驱动板 PWM_1_IN
*   方向      PA4  推挽输出    → DIR_1_IN
*   编码器 A  PB6  TIM4_CH1   → A+
*   编码器 B  PB7  TIM4_CH2   → B+
*   上位机    PA9/PA10 USART1 115200 8N1（ST-Link 虚拟串口）
*
* TIM5 1 ms 中断里只做：读编码器增量 + 每 5 拍控制一次 + 每 10 拍置上传标志。
* 串口解析和发送都在主循环，所以通信/调参不会让 PWM 更新断拍。
********************************************************************************************************************/
#ifndef _NEUQ_MOTOR_HW_H_
#define _NEUQ_MOTOR_HW_H_

#include "NEUQ_lib_include.h"

/* ============================== 接线宏 ============================== */
#define NEUQ_MOTORH_PWM_PORT        TIM2_CH3_PA2
#define NEUQ_MOTORH_PWM_FREQ        5000u           /* 库内整数分频，实际约 8.4 kHz */

#define NEUQ_MOTORH_DIR_GPIO        PA4
#define NEUQ_MOTORH_DIR_FWD_LEVEL   GPIO_High
#define NEUQ_MOTORH_DIR_REV_LEVEL   GPIO_Low

#define NEUQ_MOTORH_ENC_DEV         Encoder_TIM4
#define NEUQ_MOTORH_ENC_A           Encoder_TIM4_PhaseA_PB6
#define NEUQ_MOTORH_ENC_B           Encoder_TIM4_PhaseB_PB7

#define NEUQ_MOTORH_UART            UART_0          /* USART1：PA9/PA10 */
#define NEUQ_MOTORH_UART_REG        USART1

#define NEUQ_MOTORH_INT_PRIO_UART   1u
#define NEUQ_MOTORH_INT_PRIO_CTRL   2u

#define NEUQ_MOTORH_CTRL_TICKS      5u              /* 5 ms 控制周期 */
#define NEUQ_MOTORH_UPLINK_TICKS    10u             /* 10 ms 上传周期 */

/* ============================== 接口 ============================== */
void NEUQ_MotorHw_Init(void);

/* 占空比 + 方向一起写，避免方向未翻先给力 */
void NEUQ_MotorHw_SetOutput(float duty, uint8_t dir_forward);

/* 读走本拍编码器增量并清零（只在 TIM5 中断里调） */
int32_t NEUQ_MotorHw_EncReadClear(void);

/* 非阻塞发送：整帧入队，放不下就丢；TxPump 在主循环里按 TXE 吐字节 */
void NEUQ_MotorHw_TxFrame(const uint8_t *buf, uint16_t len);
void NEUQ_MotorHw_TxPump(void);

uint32_t NEUQ_MotorHw_Millis(void);

/* 接收队列：主循环按字节取 */
uint8_t  NEUQ_MotorHw_RxPop(uint8_t *byte);
uint32_t NEUQ_MotorHw_RxDrop(void);
uint32_t NEUQ_MotorHw_TxDrop(void);

/* USART1 接收中断回调只入队 */
void NEUQ_MotorApp_RxByte(uint8_t byte);

#endif /* _NEUQ_MOTOR_HW_H_ */
