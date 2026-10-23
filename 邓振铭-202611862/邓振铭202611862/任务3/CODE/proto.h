#ifndef _PROTO_H_
#define _PROTO_H_
#include"NEUQ_lib_include.h"

#define PROTO_HEAD 0xAA
#define PROTO_TAIL 0xBB
#define CMD_MOTOR_STOP 0x00
#define CMD_MOTOR_RUN 0x01
#define CMD_SET_TAR_SPD 0x02
#define CMD_SET_KP 0x03
#define CMD_SET_KI 0x04
#define CMD_SET_KD 0x05
void NEUQ_Protoco_Init(void);
void NEUQ_Protoco_Analysis(uint8_t dat);
void NEUQ_Protoco_Push(uint8_t dat);   /* 接收中断调用，仅入 FIFO */
void NEUQ_Protoco_Task(void);

extern float proto_tar_spd;
extern float proto_kp;
extern float proto_ki;
extern float proto_kd;

#endif