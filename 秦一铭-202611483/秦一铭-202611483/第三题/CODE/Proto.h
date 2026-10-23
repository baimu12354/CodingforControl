#ifndef _PROTO_H_
#define _PROTO_H_

#include "NEUQ_lib_include.h"

extern volatile uint8_t  g_run;          // 0=停止（上电默认） 1=运行
extern volatile int16_t  g_target_rpm;   // 当前生效目标转速
extern volatile int16_t  g_actual_rpm;   // 实际转速（Ctrl_Tick 每 5ms 更新）
extern volatile int16_t  g_pid[3];       // Kp、Ki、Kd（实际值 ×100）

void NEUQ_Proto_Init(void);              // 串口 + FIFO + 接收中断 + 1ms 节拍
void NEUQ_Proto_RxByte(uint8_t dat);     // 串口中断里调用：只存不发
void NEUQ_Proto_Tick(void);              // 1ms 中断里调用
void NEUQ_Proto_Task(void);              // 主循环里调用：解析命令 + 回传

#endif

/* ═══════ 串口字节协议 ═══════
   帧格式：
        AA | LEN | CMD | DATA | SUM | BB
        LEN = CMD + DATA 的字节数（LEN=1 表示只有 CMD）
        SUM = (LEN + CMD + ΣDATA) 
   命令：
        0x01 START       DATA = 目标转速 int16（rpm）
        0x02 STOP        无 DATA
        0x03 SET_TARGET  DATA = 目标转速 int16（rpm），运行中可改
        0x04 SET_PID     DATA = Kp,Ki,Kd 各 int16（×100），只允许在停止状态改
   ═══════════════════════════════════ */
