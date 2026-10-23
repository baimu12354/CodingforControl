#ifndef _VOFA_H_
#define _VOFA_H_
#include "NEUQ_lib_include.h"

/* JustFloat 通道定义 */
#define VOFA_JF_CH_COUNT   8
#define VOFA_JF_CH_TGT0    0
#define VOFA_JF_CH_ACT0    1
#define VOFA_JF_CH_TGT1    2
#define VOFA_JF_CH_ACT1    3
#define VOFA_JF_CH_KP      4
#define VOFA_JF_CH_KI      5
#define VOFA_JF_CH_KD      6
#define VOFA_JF_CH_EN      7

void VOFA_Init(void);
void VOFA_RxByte(uint8_t byte);
void VOFA_Task(void);

#endif