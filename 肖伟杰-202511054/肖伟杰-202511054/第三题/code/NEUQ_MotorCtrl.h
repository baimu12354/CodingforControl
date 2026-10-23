/*********************************************************************************************************************
* 文件名称          NEUQ_MotorCtrl
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第三题：编码器测速与单位换算 + 位置式 PID + 输出限幅 + 条件积分抗饱和
*
* 测速：TIM4 硬件正交编码器，13 线 × 4 倍频 × 减速比 30 = 1560 计数 / 输出轴圈。
*       1 ms 中断里读增量，5 ms 控制拍用 20 ms 滑动窗口求和：
*           RPM = 窗口计数 × 60000 / (1560 × 20) = 窗口计数 × 1.9231
*       再叠一级一阶低通（α = 0.25）压量化台阶。
*
* 控制律：out = Kp·e + Ki·Σe + Kd·(0 − Δ实测)
*   - 微分项用实测变化量，目标突跳不会踢出 Kd×Δ目标 的尖峰；
*   - 积分用条件积分：输出已经饱和且误差还想往同方向推时停止累积；
*   - 输出限幅 ±out_max，负输出按 0 处理（本车只要求单向调速）。
********************************************************************************************************************/
#ifndef _NEUQ_MOTOR_CTRL_H_
#define _NEUQ_MOTOR_CTRL_H_

#include <stdint.h>

/* ============================== 机械与测速参数 ============================== */
#define NEUQ_MOTOR_ENC_LINES         13.0f      /* 编码器线数 */
#define NEUQ_MOTOR_ENC_X4            4.0f       /* 正交 4 倍频 */
#define NEUQ_MOTOR_ENC_CPR           (NEUQ_MOTOR_ENC_LINES * NEUQ_MOTOR_ENC_X4)
#define NEUQ_MOTOR_GEAR_RATIO        30.0f      /* 电机轴 : 输出轴 */
#define NEUQ_MOTOR_COUNTS_PER_REV    (NEUQ_MOTOR_ENC_CPR * NEUQ_MOTOR_GEAR_RATIO)

/* 编码器方向：电机正转（输出 duty 为正）时实测转速应为正；接反就改成 -1.0f */
#define NEUQ_MOTOR_ENC_DIR_SIGN      1.0f

/* 运行中改 PID 时保持积分项出力连续，避免调参瞬间掉速 */
#ifndef NEUQ_MOTOR_PID_BUMPLESS
#define NEUQ_MOTOR_PID_BUMPLESS      1
#endif

/* 测速滑动窗口（ms）与一阶低通系数 */
#ifndef NEUQ_MOTOR_SPEED_WIN_MS
#define NEUQ_MOTOR_SPEED_WIN_MS      20u
#endif
#ifndef NEUQ_MOTOR_SPEED_LPF_ALPHA
#define NEUQ_MOTOR_SPEED_LPF_ALPHA   0.25f
#endif

/* ============================== 默认 PID 与限幅 ============================== */
#define NEUQ_MOTOR_DEF_KP            20.0f      /* 占空比单位：10000 = 100% */
#define NEUQ_MOTOR_DEF_KI            3.0f
#define NEUQ_MOTOR_DEF_KD            80.0f

#define NEUQ_MOTOR_RPM_LIMIT         2000.0f    /* 目标转速上限 */
#define NEUQ_MOTOR_GAIN_LIMIT        1000.0f    /* Kp/Ki/Kd 上限 */
#define NEUQ_MOTOR_OUT_MAX           9000.0f    /* PWM 限幅 */
#define NEUQ_MOTOR_INT_MAX           20000.0f   /* 积分累加限幅 */

/* ============================== 结构体 ============================== */
typedef struct
{
    float    duty;                          /* 带符号占空比（本工程只输出非负） */
    float    duty_abs;                      /* |duty|，直接写 PWM 比较寄存器 */
    uint8_t  dir_forward;                   /* 方向脚：1 = 正转 */
    float    rpm_meas;                      /* 滤波后实测转速（PID 用） */
    float    rpm_raw;                       /* 窗口原始转速（上传用） */
} NEUQ_MotorCtrlOutputType;

typedef struct
{
    NEUQ_MotorCtrlOutputType out;

    float Kp;
    float Ki;
    float Kd;
    float target_rpm;                       /* 下发目标（回传 ch0） */
    float out_max;
    float int_max;

    float Ek_total;                         /* 积分累加 Σe */
    float integral;                         /* Ki·Σe，无扰切换用 */
    float d_term;                           /* 本拍微分项 */
    float rpm_prev;                         /* 上一拍实测转速 */

    uint8_t lpf_init;                       /* 测速低通是否已吃第一个样本 */
    int32_t d_counts[NEUQ_MOTOR_SPEED_WIN_MS];
    int32_t d_sum;                          /* 滑动窗口计数和 */
    uint8_t d_idx;
    float   win_ms;
    uint8_t run;                            /* 1 = 闭环运行中 */
} NEUQ_MotorCtrlType;

/* ============================== 接口 ============================== */
void  NEUQ_MotorCtrl_Init(NEUQ_MotorCtrlType *c);
void  NEUQ_MotorCtrl_Reset(NEUQ_MotorCtrlType *c);
void  NEUQ_MotorCtrl_SetPid(NEUQ_MotorCtrlType *c, float kp, float ki, float kd);
void  NEUQ_MotorCtrl_SetTarget(NEUQ_MotorCtrlType *c, float rpm);
void  NEUQ_MotorCtrl_Start(NEUQ_MotorCtrlType *c);
void  NEUQ_MotorCtrl_Stop(NEUQ_MotorCtrlType *c);
void  NEUQ_MotorCtrl_FeedTick(NEUQ_MotorCtrlType *c, int32_t delta_counts);
void  NEUQ_MotorCtrl_ControlTick(NEUQ_MotorCtrlType *c);

#endif /* _NEUQ_MOTOR_CTRL_H_ */
