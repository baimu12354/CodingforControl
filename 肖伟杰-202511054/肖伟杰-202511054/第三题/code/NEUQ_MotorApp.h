/*********************************************************************************************************************
* 文件名称          NEUQ_MotorApp
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第三题：应用层（初始化装配 + 1 ms 节拍调度 + 主循环：解包 / 生效 / 上传）
*
* ================================ 任务怎么分工（对应题目第 3 条） ================================
*   TIM5 中断（1 ms，优先级 2）—— "高精度控制放在简短的定周期中断"：
*       NeUQ_MotorHw_EncReadClear()   读编码器计数增量并清零
*       NEUQ_MotorApp_Tick()          每 5 拍：测速换算 + PID + 写 PWM；每 10 拍：置上传标志
*   主循环 —— "串口解析和上传安排在主循环等适当位置"：
*       NEUQ_MotorApp_Task()          ① 发送泵（TXE 就绪才写，不等待）
*                                     ② 半帧超时重同步
*                                     ③ 把接收队列里的字节全部喂进解析器（有几帧解几帧）
*                                     ④ 到点发一帧 FireWater 曲线
*                                     ⑤ 每 100 ms 查一次串口溢出，兜底恢复
*   两边只通过"环形队列 + 几个 volatile 标志"交换数据，谁都不会等谁：
*   通信与调参期间 PWM 更新一拍都不会断。
*
* ================================ 与 NEUQ 库/工程的接法（一共只有 3 处） ================================
*   ① User/NEUQ_Main.c        的 NEUQ_APP_SELECT == 3 分支：调用 NEUQ_MotorApp_Main();
*   ② User/NEUQ_Interrupts.c  TS_UART_0_RX_CALLBACK(dat)：调用 NEUQ_MotorApp_RxByte(dat);
*   ③ CODE/NEUQ_AppSelect.h   的 NEUQ_APP_SELECT 改成 3
*   其余全部在本文件与 NEUQ_MotorHw 里，不需要动库。
********************************************************************************************************************/
#ifndef _NEUQ_MOTOR_APP_H_
#define _NEUQ_MOTOR_APP_H_

#include <stdint.h>
#include "NEUQ_MotorCtrl.h"

/* 全局唯一的控制器实例：硬件层、命令层、上传层看到的都是它 → 回传值就是实际在用的值 */
extern NEUQ_MotorCtrlType g_motor_ctrl;

/* 第三题入口（User/NEUQ_Main.c 调用）：初始化 + 进死循环，不会返回 */
void NEUQ_MotorApp_Main(void);

/* 1 ms 节拍（TIM5 中断里调用），delta 是本拍读到的编码器计数增量 */
void NEUQ_MotorApp_Tick(int32_t delta);

/* 主循环任务 */
void NEUQ_MotorApp_Task(void);

/* 累计编码器计数（带符号；诊断与"每圈计数"标定用，ch10 回传的就是它） */
extern volatile int32_t g_enc_total;

/* 串口接收中断回调（User/NEUQ_Interrupts.c 调用）：只入队 */
void NEUQ_MotorApp_RxByte(uint8_t byte);

#endif /* _NEUQ_MOTOR_APP_H_ */
