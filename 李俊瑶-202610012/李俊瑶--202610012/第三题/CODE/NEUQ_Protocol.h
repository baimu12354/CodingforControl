#ifndef _NEUQ_PROTOCOL_H_
#define _NEUQ_PROTOCOL_H_

#include <stdint.h>

/* Frame: AA LEN CMD DATA BB; LEN is the DATA byte count. */
#define PROTO_HEAD 0xAA
#define PROTO_TAIL 0xBB
#define CMD_START_STOP 0x01 // LEN=1，01 启动，00 停止
#define CMD_SET_SPEED  0x02 // LEN=4，小端 float，与反馈同单位，非负
#define CMD_SET_PID    0x03 // LEN=12，小端 float：Kp、Ki、Kd，仅停机时生效
#define CMD_SET_KP     0x04 // LEN=4，小端 float：Kp，仅停机时生效
#define CMD_SET_KI     0x05 // LEN=4，小端 float：Ki，仅停机时生效
#define CMD_SET_KD     0x06 // LEN=4，小端 float：Kd，仅停机时生效

typedef enum {
    WAIT_HEAD, WAIT_LEN, WAIT_CMD, WAIT_DATA, WAIT_TAIL
} ProtoState;

void NEUQ_Protocol_Init(void);
void NEUQ_Protocol_Receive(uint8_t dat);
void NEUQ_Protocol_Analysis(uint8_t dat);
void NEUQ_Protocol_Task(void);
void NEUQ_Protocol_Report(void);

#endif
