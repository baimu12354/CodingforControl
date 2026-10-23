#ifndef __NEUQ_PROTO_H_
#define __NEUQ_PROTO_H_

#include "NEUQ_lib_include.h"

//协议格式：五帧格式——头帧+指令帧+长度帧+数据帧+尾帧

//状态机
typedef enum
{
    Head=0,
    Cmd,
    Len,
    Data,
    Tail,

} NEUQ_PROTO_State;

//常量
#define DATA_MAX_LEN    20
#define PROTO_UART UART_0
#define PROTO_UPLOAD_MS 1
static NEUQ_FIFO_t NEUQ_PROTO_FIFO;
static uint8_t PROTO_FIFO_buffer[256];
static NEUQ_PROTO_State state=Head ;
static uint8_t cmd=0;
static uint8_t len=0;
static uint8_t data_cnt=0;
static uint8_t data[DATA_MAX_LEN];
static int32_t last_upload_ms;


//帧常量，命令帧
#define PROTO_HEAD      0xAA
#define PROTO_TAIL      0xBB
#define PROTO_CMD_START 0x00
#define PROTO_CMD_END   0x01
#define PROTO_CMD_PID_P 0X02
#define PROTO_CMD_PID_I 0X03    
#define PROTO_CMD_PID_D 0X04
#define PROTO_CMD_RPM   0X05  



void NEUQ_PROTO_Init(void);
void NEUQ_PROTO_Push(uint8_t data);
void NEUQ_PROTO_Task(void);

#endif