/*********************************************************************************************************************
* 文件名称          code
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
********************************************************************************************************************/
#ifndef __CODE_H__
#define __CODE_H__

#include <stdint.h>

// 毫秒时基：由 NEUQ_Interrupts.c 中 NEUQ_PIT_1_CALLBACK 每 1ms 自增
extern volatile uint32_t Tick_ms;

void Code_Init(void);   // 初始化外设（LED / 按键 / LCD / PIT）
void Code_Task(void);   // 主循环任务（LED 时序 + 按键切换 + LCD 刷新）

#endif
