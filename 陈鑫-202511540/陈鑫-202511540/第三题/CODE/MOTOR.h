#ifndef MOTOR_H
#define MOTOR_H

#include "NEUQ_lib_include.h"

/* ==================== 硬件配置 ==================== */
#define MOTOR_PWM_PIN      TIM8_CH1_PC6
#define MOTOR_DIR_PIN      PC7
#define MOTOR_DIR_FWD      GPIO_Low
#define MOTOR_DIR_REV      GPIO_High

#define ENCODER_DIR_SIGN   1

/* 驱动板 EN/STBY */
#define MOTOR_USE_EN       1
#define MOTOR_EN_PIN       PB13
#define MOTOR_EN_ON        GPIO_High

/* 编码器：PB6=A相 PB7=B相（TIM4 编码器接口） */
#define ENCODER_DEV        Encoder_TIM4
#define ENCODER_PHASE_A    Encoder_TIM4_PhaseA_PB6
#define ENCODER_PHASE_B    Encoder_TIM4_PhaseB_PB7

/* 控制周期：必须与 PIT 中断周期一致 */
#define CTRL_PERIOD_MS     10

/* ==================== 对外变量 ==================== */
/* 速度量纲：脉冲/10ms（正=正转 负=反转） */
extern int32_t  spd_aim;       /* 目标速度（可正可负） */
extern int32_t  spd_now;       /* 实际速度（可正可负） */
extern int16_t  duty_now;      /* 当前占空比：±10000 */

extern float    pid_kp;
extern float    pid_ki;
extern float    pid_kd;

extern uint8_t  sys_running;   /* 1=运行  0=停机 */

/* 编码器累计计数（10ms 中断自动累计，停机也跑，手转标定 PPR 用） */
extern volatile int32_t  enc_total_cnt;

/* 编码器方向符号：1=不翻转 -1=翻转（A/B 接反时改 -1） */
extern volatile int8_t   enc_dir_sign;

/* 阶跃响应时间(ms)：到目标 90% 记用时，5s 没到记 -1 */
extern volatile int32_t  step_rise_ms;

/* ==================== 函数声明 ==================== */
void motor_init(void);
void motor_setduty(int duty);
void motor_ctrl(void);
void motor_pid_reset(void);

#endif /* MOTOR_H */