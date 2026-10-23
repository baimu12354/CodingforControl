#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

typedef enum
{
    MOTOR_CONTROL_FAULT_NONE = 0U,
    MOTOR_CONTROL_FAULT_NO_ENCODER = 1U,
    MOTOR_CONTROL_FAULT_WRONG_DIRECTION = 2U
} Motor_Control_Fault;

typedef struct
{
    float target_rpm;
    float actual_rpm;
    float kp;
    float ki;
    float kd;
    uint8_t running;
    uint8_t fault;
    int32_t encoder_total_count;
} Motor_Control_Status;

/* 初始化电机方向、PWM 和编码器，上电保持停机。 */
void Motor_Control_Init(void);

/* 由 10 ms PIT 中断调用：测速、PID 计算、PWM 更新。 */
void Motor_Control_Update10ms(void);

/* 主循环中的串口命令只通过这些接口修改控制状态。 */
void Motor_Control_SetRunning(uint8_t running);
void Motor_Control_SetTargetRpm(float target_rpm);
void Motor_Control_SetPid(float kp, float ki, float kd);

uint8_t Motor_Control_IsRunning(void);
uint8_t Motor_Control_HasFault(void);
void Motor_Control_ClearFault(void);
void Motor_Control_ResetEncoderTotal(void);
void Motor_Control_GetStatus(Motor_Control_Status *status);

#endif /* MOTOR_CONTROL_H */
