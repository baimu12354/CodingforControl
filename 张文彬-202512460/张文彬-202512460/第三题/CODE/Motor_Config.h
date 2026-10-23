#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "NEUQ_PWM.h"
#include "NEUQ_GPIO.h"
#include "NEUQ_Encoder.h"
#include "NEUQ_PIT.h"
#include "NEUQ_Uart.h"

/* ==================== 电机 1 硬件连接 ====================
 * 主板 H11 -> 驱动板 H1：
 * PC6 -> PWM_1_IN，PC7 -> DIR_1_IN
 * PA6/PA7 -> 当前这一路电机的编码器 A/B 相
 * 编码器电源和主板、驱动板必须共地。
 */
/* 恢复为最早已在实机上验证能驱动该接口的 PC6 / TIM8_CH1 复用。 */
#define MOTOR_PWM_PORT                         TIM8_CH1_PC6
#define MOTOR_PWM_HZ                           1000UL
#define MOTOR_PWM_TIMER_CLOCK_HZ               84000000UL
#define MOTOR_PWM_INITIAL_DUTY                 0U
#define MOTOR_DIRECTION_PIN                    PC7

/* 实机已验证：PC6/PC7 驱动的电机反馈来自 TIM3 的 PA6/PA7。 */
#define MOTOR_ENCODER                          Encoder_TIM3
#define MOTOR_ENCODER_PHASE_A                  Encoder_TIM3_PhaseA_PA6
#define MOTOR_ENCODER_PHASE_B                  Encoder_TIM3_PhaseB_PA7

/* ==================== 闭环控制参数 ==================== */
#define MOTOR_CONTROL_PIT                      PIT_CH1
#define MOTOR_CONTROL_PERIOD_MS                10UL
#define MOTOR_CONTROL_DT                       ((float)MOTOR_CONTROL_PERIOD_MS / 1000.0f)

/* 11 PPR、10:1 减速、AB 两相四倍频：11 * 10 * 4 = 440。 */
#define MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV    440.0f

/* 正向驱动时 TIM3 原始计数为负，统一翻转累计计数和转速符号。 */
#define MOTOR_ENCODER_COUNT_DIRECTION          (-1)
#define MOTOR_ENCODER_DIRECTION                ((float)MOTOR_ENCODER_COUNT_DIRECTION)

/* 一阶低通：越小越平滑但延迟越大；调参时主要修改这一项。 */
#define MOTOR_SPEED_FILTER_ALPHA               0.25f

#define MOTOR_MAX_TARGET_RPM                   1000.0f
#define MOTOR_TARGET_ZERO_DEADBAND_RPM         0.5f
#define MOTOR_PID_INTEGRAL_LIMIT               5000.0f

/*
 * 当前是单电机台架考核，默认不让诊断逻辑自动切断 PWM。
 * 之前的 500 ms 无反馈保护会在电机尚未起转/驱动板未响应时锁存停机：
 * I5 会从 1 变 0、I9 变 1，之后再次 START 也会被拒绝。
 * 等电机能够稳定驱动后，可把下面两项改成 1U 重新启用保护。
 */
#define MOTOR_ENABLE_NO_ENCODER_AUTO_STOP      0U
#define MOTOR_ENABLE_WRONG_DIR_AUTO_STOP       0U

/* 有较大输出却长时间没有编码器脉冲时的诊断阈值。 */
#define MOTOR_ENCODER_FAULT_MIN_TARGET_RPM     100.0f
#define MOTOR_ENCODER_FAULT_MIN_DUTY           2000.0f
#define MOTOR_ENCODER_FAULT_TIMEOUT_MS         500UL
#define MOTOR_ENCODER_FAULT_MIN_COUNTS         4UL

/* 编码器方向与目标方向相反时的诊断阈值。 */
#define MOTOR_DIRECTION_FAULT_MIN_RPM          50.0f
#define MOTOR_DIRECTION_FAULT_TIMEOUT_MS       200UL

/* 按 12 V、1000 RPM 铭牌做基础占空比前馈，PID 负责修正负载和误差。 */
#define MOTOR_NOMINAL_NO_LOAD_RPM              1000.0f
#define MOTOR_ENABLE_SPEED_FEEDFORWARD         1U

#define MOTOR_DEFAULT_KP                       2.0f
#define MOTOR_DEFAULT_KI                       0.8f
#define MOTOR_DEFAULT_KD                       0.01f

/* ==================== 单串口与 VOFA ==================== */
#define MOTOR_UART                             UART_0
#define MOTOR_UART_BAUD                        UART_BAUD_115200
#define MOTOR_UART_INTERRUPT_PRIORITY          5U
#define MOTOR_VOFA_PERIOD_MS                    20UL
#define MOTOR_VOFA_CHANNEL_COUNT               11U

/* 学校 PWM 封装固定 ARR=PWM_MAX-1，必须保留 0~10000 的占空比满量程。 */
#if (PWM_MAX != 10000)
#error "This project requires PWM_MAX=10000."
#endif

#if (MOTOR_PWM_INITIAL_DUTY > PWM_MAX)
#error "MOTOR_PWM_INITIAL_DUTY must not exceed PWM_MAX."
#endif

/* 频率过高会令学校 PWM 封装的预分频先变成 0，随后发生无符号下溢。 */
#if (MOTOR_PWM_HZ == 0UL) || (MOTOR_PWM_HZ > (MOTOR_PWM_TIMER_CLOCK_HZ / PWM_MAX))
#error "MOTOR_PWM_HZ is outside the valid range of the NEUQ PWM wrapper."
#endif

#if (MOTOR_CONTROL_PERIOD_MS == 0UL)
#error "MOTOR_CONTROL_PERIOD_MS must be greater than zero."
#endif

#if (MOTOR_ENCODER_FAULT_TIMEOUT_MS < MOTOR_CONTROL_PERIOD_MS) || \
    ((MOTOR_ENCODER_FAULT_TIMEOUT_MS % MOTOR_CONTROL_PERIOD_MS) != 0UL)
#error "MOTOR_ENCODER_FAULT_TIMEOUT_MS must be a multiple of the control period."
#endif

#if (MOTOR_DIRECTION_FAULT_TIMEOUT_MS < MOTOR_CONTROL_PERIOD_MS) || \
    ((MOTOR_DIRECTION_FAULT_TIMEOUT_MS % MOTOR_CONTROL_PERIOD_MS) != 0UL)
#error "MOTOR_DIRECTION_FAULT_TIMEOUT_MS must be a multiple of the control period."
#endif

#define MOTOR_ENCODER_FAULT_WINDOW_CYCLES      \
    (MOTOR_ENCODER_FAULT_TIMEOUT_MS / MOTOR_CONTROL_PERIOD_MS)
#define MOTOR_DIRECTION_FAULT_WINDOW_CYCLES    \
    (MOTOR_DIRECTION_FAULT_TIMEOUT_MS / MOTOR_CONTROL_PERIOD_MS)

#endif /* MOTOR_CONFIG_H */
