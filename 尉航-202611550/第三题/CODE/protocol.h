#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "NEUQ_lib_include.h"
/* ========== 帧格式：AA | LEN | CMD | 负载[LEN] | CHECK | BB ========== */
#define PROTO_HEAD  0xAA
#define PROTO_TAIL  0xBB

#define CMD_START   0x00    // 启动电机
#define CMD_OFF     0x01    // 关闭电机
#define CMD_UPDATE  0x03    // 更新目标
#define CMD_PID     0x04    // 一次设置 Kp/Ki/Kd
#define CMD_KP      0x05    // 改 Kp
#define CMD_KI      0x06    // 改 Ki
#define CMD_KD      0x07    // 改 Kd


#define CTRL_PERIOD_US     10000        // 10ms一次PID中断
#define CNT_MAX_PER_MS     60           // 满速时每次PID中断周期最大计数
#define FEEDBACK_K         (PWM_MAX / CNT_MAX_PER_MS)


#define MACHINE_PORT TIM2_CH3_PA2      
#define DIR_PORT     PA5              

void protocol_init(void);
void protocol_analysis(uint8_t dat);
void protocol_task(void);
void control_loop(void);
void sendvofa(void);

#endif