#ifndef _CMD_H_
#define _CMD_H_

#include "NEUQ_lib_include.h"

#define CMD_HEAD_ONE 0XAA
#define CMD_HEAD_TWO 0X55
#define CMD_START 0x01
#define CMD_PUSE 0x02
#define CMD_SET_TARGET 0x03
#define CMD_SET_PID 0x04
#define CMD_SEND 0X05


extern uint8_t buf[256];
extern NEUQ_FIFO_t fifo; 
extern uint8_t send_buf[256]; 
extern NEUQ_FIFO_t send_fifo; 


void Cmd_Inite(void);

void Cmd_UnpackData(void);

void Cmd_SendData(void);


// void Cmd_Send(int8_t,int8_t);//需要上位机编辑，先用printf代替


#endif 