#ifndef _NEUQ_Motor_
#define _NEUQ_Motor_
#include "NEUQ_lib_include.h"

// ==================== 硬件配置（按你的实际接线修改！）====================
// 电机驱动：一路 PWM 调速 + 一路 GPIO 方向脚 DIR
#define MOTOR_PWM_PIN      TIM2_CH3_PA2   // 电机 PWM 调速引脚
#define MOTOR_DIR_PIN      PC6            // 电机方向脚
#define MOTOR_DIR_FORWARD  GPIO_High      // 正转时方向脚的电平；如果方向反了，把 High 改成 Low
#define MOTOR_DIR_REVERSE  ((MOTOR_DIR_FORWARD == GPIO_High) ? GPIO_Low : GPIO_High)

// 编码器
#define ENCODER_DEV        Encoder_TIM4
#define ENCODER_PHASE_A    Encoder_TIM4_PhaseA_PB6
#define ENCODER_PHASE_B    Encoder_TIM4_PhaseB_PB7
#define ENCODER_PPR        13             // 编码器线数（每圈脉冲数，看你的编码器型号）
#define ENCODER_DIR_SIGN   1              // 编码器读数方向；如果测出来转速正负反了，改成 -1

// 控制参数
#define CONTROL_PERIOD_MS  10             // 控制周期 10ms（100Hz）
#define PWM_FREQ           1000           // PWM 频率 1kHz

// 供协议层回传读取的实际转速（在控制中断里更新）
extern volatile float motor_actual_speed;
// 系统节拍，每 CONTROL_PERIOD_MS 毫秒累加一次，供主循环定时回传用
extern volatile uint32_t motor_tick_ms;

void NEUQ_Motor_Init(void);
void NEUQ_Motor_Control_Handler(void);   // 放在 PIT1 中断里调用

#endif
