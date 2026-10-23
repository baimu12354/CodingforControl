#ifndef _NEUQ_PACK_H_
#define _NEUQ_PACK_H_
#include "NEUQ_lib_include.h"

//编码器相关数据，用于单位转化
#define ENCODER_LINES   11
#define ENCODER_MULT    4
#define GEAR_RATIO      10
#define CPR_OUTPUT      (ENCODER_LINES * ENCODER_MULT * GEAR_RATIO)   // 440
#define RPM_TO_CNT10MS(r)   ((float)(r) * (float)CPR_OUTPUT / 6000.0f)
#define CNT10MS_TO_RPM(c)   ((float)(c) * 6000.0f / (float)CPR_OUTPUT)


typedef enum{
    CMD_START = 0x01,   // 启动
    CMD_STOP  = 0x02,   // 停止
    CMD_SPEED = 0x03,   // 目标转速（int16）
    CMD_PID   = 0x04,   // PID 参数（3×float）
}CMDs;

typedef enum{
    RX_H1,
    RX_H2,
    RX_CMD,
    RX_LEN,
    RX_DATA,
    RX_CHK
}RX_NUM;

extern NEUQ_FIFO_t rx_fifo;
extern volatile uint8_t upload_flag;

void NEUQ_Pack_init(void);
void recv_update(void);//读取+更新参数
void vofa_send(void); 

#endif