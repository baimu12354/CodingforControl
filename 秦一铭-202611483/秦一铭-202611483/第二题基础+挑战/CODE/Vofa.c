#include "NEUQ_lib_include.h"

int32_t send_axis = 0;      // 默认发陀螺 X
int32_t send_on   = 0;      // 默认不发送，由菜单打开

static uint8_t flag = 0;                                // 发一帧标志
static uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};      // JustFloat 帧尾

// 中断里每 2ms 调一次，5*2=10ms，即 100Hz
void Vofa_Tick(void){
    static uint8_t div = 0;

    if (++div < 5) return;
    div = 0;
    flag = 1;
}

// 主循环里调用：一帧 = 原始值 + 滤波值 + 帧尾，共 12 字节
void Vofa_Task(void){
    float frame[2];

    if (flag == 0) return;
    flag = 0;
    if (send_on == 0) return;       // 关掉时一帧都不发

    frame[0] = raw[send_axis];      // 通道 0：原始
    frame[1] = filt[send_axis];     // 通道 1：滤波

    NEUQ_Uart_write_data(UART_0, (uint8_t *)frame, 8);
    NEUQ_Uart_write_data(UART_0, tail, 4);
}