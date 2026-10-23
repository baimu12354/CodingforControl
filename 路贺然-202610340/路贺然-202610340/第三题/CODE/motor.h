#ifndef NEUQ_MOTOR_H_
#define NEUQ_MOTOR_H_

#include"NEUQ_lib_include.h"
typedef struct{
    uint8_t running;
    float target_speed;
    float actual_speed;
    float kp;
    float ki;
    float kd;
    float e0;
    float e1;
    float e2;
    float delta;
    float pid_output;      //pid输出
}motor_t;
extern motor_t g_motor;


#define MOTOR_PWM_CH    TIM2_CH1_PA5             // PWM 通道
#define MOTOR_DIR_PIN   PA2                      // 方向引脚
#define MOTOR_ENC_TIM   Encoder_TIM4             // 编码器定时器
#define MOTOR_ENC_A     Encoder_TIM4_PhaseA_PB6  // 编码器 A 相
#define MOTOR_ENC_B     Encoder_TIM4_PhaseB_PB7  // 编码器 B 相

// 编码器测速换算参数
#define MOTOR_ENC_PPR        330     // 11线 × 1:30减速比 = 输出轴等效线数（显示/控制输出轴转速）
#define MOTOR_ENC_MULT       4       // STM32 编码器接口为 4 倍频（330×4 = 1320 计数/输出轴一圈）
#define MOTOR_CTRL_PERIOD_MS 10     // 控制周期，需与 PIT 周期一致

#define PID_OUT_MAX     10000.0f


void Motor_Init(void);                          //初始化
void Motor_ControlTick(void);                   //10ms 定周期中断：测速 + 控制计算 + PWM 更新
void Motor_SetTargetSpeed(float speed);         //设置目标速度(rpm)
void Motor_SetPid(float kp,float ki,float kd);  //设置PID参数(停止时生效)
void Motor_Start(void);                         //启动电机
void Motor_Stop(void);                          //停止电机
float Motor_CountToRPM(int32_t counts);         //编码器计数 -> 转速(rpm)


extern volatile uint32_t g_motor_tick_ms;       //10ms 节拍计数，供主循环回传节拍
#endif
