#ifndef NEUQ_wave_H_
#define NEUQ_wave_H_
#include "NEUQ_lib_include.h"


void Wave_Init(void);          // 上电: UART + FIFO + Flash 加载
void Wave_Tick(void);          // PIT
void Wave_FlashSave(void);     // 主循环
void PIT_ISR(void);

extern volatile uint8_t g_wave_save_req;   // 编辑完成

#endif