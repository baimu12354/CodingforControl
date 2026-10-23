#ifndef MOTOR_H_
#define MOTOR_H_

#include "NEUQ_lib_include.h"

// 数据包协议
#define FRAME_HEAD1     0xAA    // 帧头1
#define FRAME_HEAD2     0x55    // 帧头2

#define CMD_START       0x01    // 启动电机
#define CMD_STOP        0x02    // 停止电机
#define CMD_SET_SPEED   0x03    // 设置目标转速
#define CMD_SET_KP      0x04    // 单独设置Kp
#define CMD_SET_KI      0x05    // 单独设置Ki
#define CMD_SET_KD      0x06    // 单独设置Kd

typedef struct {
    volatile uint8_t  status;       // 1:运行 0:停止
    volatile int32_t  target_rpm;   // 目标转速rpm
    volatile float    kp, ki, kd;   // 当前PID参数
    volatile float    actual_rpm;   // 实测转速rpm
    volatile int32_t  pwm_output;   // PWM输出值
    volatile uint8_t  upload_tick;  // 上传标志
    volatile uint8_t  upload_enable;// 是否开启上传(1:开启 0:停止)
} MotorControl;

extern MotorControl motor;
extern pid_value    speed_pid;

//电机输出函数
static void Motor_SetOutput(int32_t pwm);
// 电机控制
void Motor_Control(void);
// VOFA+指令实现
static void Apply_Command(uint8_t cmd, const uint8_t *data, uint8_t len);
// 解析VOFA指令
static void Parse_Byte(uint8_t a);
// 缓存区数据更新
void Motor_Process(void);
// 数据上传VOFA
void Motor_Upload(void);
//电机初始化
void Motor_Init(void);

#endif