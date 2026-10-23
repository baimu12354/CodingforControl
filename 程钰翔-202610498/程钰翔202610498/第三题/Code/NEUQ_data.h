#ifndef _NEUQ_DATA_H
#define _NEUQ_DATA_H

#include "NEUQ_lib_include.h"

/*
  本文件：字节包通信 + 编码器测速 + 定周期 PID 闭环 + 回传曲线数据。
  串口固定用 UART_0（PA9/PA10，115200 8N1），VOFA+ 也用这一个口。
上行帧格式（上位机 -> 单片机）
  固定 8 字节，小端（无校验位）：

    | [0]  | [1]  | [2] | [3] | [4]  [5]  [6]  [7] |
    | 0xAA | 0x55 | cmd | id  | value(float, 小端)  |

  帧头 0xAA 0x55 用来定界，靠"帧长固定 8 字节 + 帧头重同步"防错位

 
      把上面 6 条帧存进数据源面板的绑定命令里面（勾 HEX！！！！），
    
  下行帧格式（单片机 -> 上位机）
  VOFA+ 的 JustFloat 协议：连续 5 个 float（小端）+ 4 字节帧尾 00 00 80 7F

    通道 0 : 目标转速（当前生效值，rad/s）
    通道 1 : 实际转速（编码器测得，rad/s）
    通道 2 : Kp
    通道 3 : Ki
    通道 4 : Kd

  协议 = JustFloat，然后按上面的通道顺序看曲线。
  回传用的就是闭环里真正参与计算的那几个变量，不做任何单位换算。

单位用 rad/s：上位机发多少就是多少，发下去直接作为 PID 给定，回传也是同一个变量。
*/

/*------------------------------ 上行协议常量 ------------------------------*/
#define PACK_HEAD0          0xAAu       /* 帧头第 1 字节 */
#define PACK_HEAD1          0x55u       /* 帧头第 2 字节 */
#define PACK_LEN            8u          /* 上行帧长固定 8 字节 */

#define CMD_START           0x01u       /* 启动 */
#define CMD_STOP            0x02u       /* 停止 */
#define CMD_SET             0x03u       /* 设置参数 */
#define CMD_SET_STEP        0x04u       /* 目标转速增量：target += value */

#define ID_TARGET_SPEED     0x00u       /* 设置目标转速（rad/s） */
#define ID_KP               0x01u       /* 设置 Kp */
#define ID_KI               0x02u       /* 设置 Ki */
#define ID_KD               0x03u       /* 设置 Kd */

/*----------------------------- 全局状态变量 -----------------------------*/
extern volatile float   target_speed;    /* 当前生效目标转速（rad/s，PID 给定） */
extern volatile float   current_speed;   /* 实际转速（rad/s，编码器测得） */
extern volatile float   pwm_output;      /* 当前 PWM 输出 -10000~10000（正=正转） */
extern volatile uint8_t motor_running;   /* 0=停机（上电默认） 1=运行 */
extern IncrementalPID   speed_pid;       /* 速度 PID（结构体来自库 NEUQ_PID.h） */

/*------------------------------ 对外接口 ------------------------------*/
/* 库文件 NEUQ_Uart.h 里漏写了该函数的声明，这里补上，避免隐式声明告警 */
void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

void NEUQ_data_init(void);                /* 开机调用一次：开电机、编码器、控制定时器、串口接收 */
void NEUQ_Control(void);                  /* 10ms 定周期中断 测速 + PID + 更新 PWM */
void NEUQ_Uart_Rx_Handler(uint8_t dat);   /* 串口中断 只把字节放进环形缓冲，不做解析 */
void NEUQ_Comm_Update(void);              /* 主循环 字节流解包 + 回传曲线数据 */

#endif
