#ifndef _NEUQ_Protcol_
#define _NEUQ_Protcol_
#include "NEUQ_lib_include.h"

// 帧格式：0xAA LEN CMD DATA[LEN] CHECK 0xBB
//   CHECK = (LEN + CMD + DATA 各字节累加) 的低 8 位
#define PROTOCOL_HEAD 0xAA
#define PROTOCOL_TAIL 0xBB

// 上位机 -> 单片机
#define CMD_START     0x01    // 启动电机  LEN=0
#define CMD_STOP      0x02    // 停止电机  LEN=0
#define CMD_SET_SPEED 0x03    // 设置目标转速  LEN=2, DATA=int16(rpm) 小端
#define CMD_SET_PID   0x04    // 设置PID参数  LEN=12, DATA=Kp,Ki,Kd 三个float(小端)
#define CMD_QUERY     0x05    // 请求立即回传  LEN=0
// 单片机 -> 上位机
#define CMD_REPORT    0x81    // 回传  LEN=20, DATA=目标float+实际float+Kp+Ki+Kd

#define PROTOCOL_DATA_MAX 64

// 生效参数，供电机控制代码读取
extern volatile uint8_t motor_run;          // 0停止 1运行
extern volatile int16_t motor_target_speed; // 目标转速 rpm
extern volatile float motor_kp;             // PID 比例系数 Kp
extern volatile float motor_ki;             // PID 积分系数 Ki
extern volatile float motor_kd;             // PID 微分系数 Kd

void NEUQ_Protcol_Init(void);
void NEUQ_Protcol_explain(uint8_t dat);
void NEUQ_Protcol_task(void);
void NEUQ_Protcol_send_report(float target, float actual);

#endif
