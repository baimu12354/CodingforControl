/* motor.h - 电机接口 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f4xx_hal.h"

/* 供主循环读取的一份状态快照。回传给上位机的就是这里的值，
 * 所以上位机显示的一定是单片机里真正在生效的参数。 */
typedef struct {
    uint8_t enabled;        /* 1 = 正在运行 */
    float   target_rpm;     /* 当前生效的目标转速 */
    float   actual_rpm;     /* 实测转速 */
    float   kp;             /* 当前生效的 PID 参数 */
    float   ki;
    float   kd;
    int32_t pwm_out;        /* 当前 PWM 输出，带符号表示方向 */
    int32_t counts_last;    /* 上一个控制周期的编码器原始计数（用于标定每转计数） */
} motor_snapshot_t;

void    Motor_Init(void);

/* 由 TIM7 中断每 10 ms 调用一次 */
void    Motor_ControlIsr(void);

/* 以下四个由主循环调用 */
void    Motor_Start(void);
void    Motor_Stop(void);
void    Motor_SetTargetRpm(float rpm);
void    Motor_SetPid(float kp, float ki, float kd);

void    Motor_GetSnapshot(motor_snapshot_t *out);

/* 主循环查询是否正在运行（用于判断能不能改 PID） */
uint8_t Motor_IsEnabled(void);

#endif /* _MOTOR_H_ */
