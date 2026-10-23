#ifndef   _cmd_control_H
#define   _cmd_control_H

#define   UART_CMD_CONTROL  USART1
#define   UART_HEADER        0xAA
#define   UART_tail          0x55
#define   UART_Check         0xFF

#include "NEUQ_lib_include.h"
void cmd_control_init(uint8_t dat);

#endif
