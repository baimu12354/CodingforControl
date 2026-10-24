#ifndef __VOFASTR_H
#define __VOFASTR_H

#include "NEUQ_lib_include.h"

void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

//单行最长字节数
#define       VOFA_LINE_MAX        32

//命令类型
typedef enum{
    TYPE_INT,
    TYPE_FLOAT
}CmdType;

//命令表项
typedef struct{
    const char *name;
    void *ptr;
    CmdType type;
}CmdItem;

void NEUQ_VOFA_Init(void);
uint8_t NEUQ_VOFA_Parse_Command(char *str);
void NEUQ_VOFA_rx_byte(uint8_t data);
void NEUQ_VOFA_task(void);

#endif