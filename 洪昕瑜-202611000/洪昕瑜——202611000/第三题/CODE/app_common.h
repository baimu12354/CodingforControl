#ifndef APP_COMMON_H
#define APP_COMMON_H

#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "config.h"

/* 共享类型 */
typedef struct { float Kp, Ki, Kd, integral, prev_actual; } PID_t;

/* 共享变量*/
extern volatile uint8_t  running;        // 运行标志              (motor_ctrl.c)
extern volatile float    target_rpm;     // 目标转速              (motor_ctrl.c)
extern volatile float    actual_rpm;     // 实际转速(低通后)       (motor_ctrl.c)
extern PID_t             pid;            // PID 参数与状态         (motor_ctrl.c)
extern volatile int32_t  pwm_duty;       // 当前占空比(±PWM_MAX, 带符号) (motor_hw.c)
extern volatile float    supply_voltage; // 供电电压(V)            (motor_hw.c)
extern volatile uint32_t time_tick;      // 1ms 时基(在 1ms PIT 中断里累加) (motor_ctrl.c)

#endif
