#ifndef _NEUQ_PROTOCO_H_
#define _NEUQ_PROTOCO_H_

#include"NEUQ_lib_include.h"

#define PROTO_HEAD 0xAA
#define PROTO_TAIL 0xBB
#define CMD_OFF    0x00
#define CMD_SINGE  0x01
#define CMD_DOUBLE 0x02
#define CMD_BREATH 0x03

#define LED_PORT TIM1_CH1_PA8

void NEUQ_Protoco_Init(void);
void NEUQ_Protoco_Analysis(uint8_t dat);
void NEUQ_Protoco_Task(void);


#endif