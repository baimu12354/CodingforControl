#ifndef __UART_PACKET_H__
#define __UART_PACKET_H__

#include "stm32f4xx_hal.h"

// ==================== 帧协议（自定义，需与上位机一致） ====================
#define FRAME_HEAD1  0xAA               // 帧头1
#define FRAME_HEAD2  0x55               // 帧头2

#define CMD_START    0x01               // 启动
#define CMD_STOP     0x02               // 停止
#define CMD_SPEED    0x03               // 设目标转速（float 小端，4字节）
#define CMD_PID      0x04               // 设 PID 参数（3 个 float 小端，12字节）
#define CMD_REPORT   0x05               // 上报（MCU→上位机，回传 5 个参数）
#define REPORT_DATA_LEN (2+4+4+4+4)     // 上报数据长：目标(2)+kp/ki/kd(各4)+速度(4)

#define MAX_DATA_LEN 32
// =====================================================================

// ==================== 对外接口 ====================
void UartPacket_Init(void);             // 初始化：使能 UART0 接收中断（调一次）
void UartPacket_RxByte(uint8_t dat);    // 中断入口：把收到的字节塞进缓冲
void UartPacket_Task(void);             // 主循环任务：解包并更新全局变量
void UartPacket_VofaSend(void);         // VOFA JustFloat：发 5 个 float（目标/kp/ki/kd/实际转速）
// ================================================

// ==================== 全局变量（用户直接读写） ====================
extern volatile uint8_t  run_flag;      // 0=停止 1=运行
extern volatile int16_t  target_rpm;    // 目标转速 (rpm)
extern volatile float    kp;            // PID 比例
extern volatile float    ki;            // PID 积分
extern volatile float    kd;            // PID 微分
extern volatile uint8_t  send_flag;     // 上报标志：1=需要发送一次
extern volatile float    actual_rpm;    // 实际转速 (rpm)
extern volatile int32_t  duty_out;      // 调试：当前占空比（-10000~10000）
// ============================================================

#endif
