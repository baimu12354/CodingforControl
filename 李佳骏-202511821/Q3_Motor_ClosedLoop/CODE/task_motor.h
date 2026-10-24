/* task_motor.h - 第三题任务接口 */

#ifndef _TASK_MOTOR_H_
#define _TASK_MOTOR_H_

void Task_Motor_Init(void);
void Task_Motor_Loop(void);

/* 由 TIM7 中断每 10 ms 调用一次：累加控制节拍并跑一次闭环 */
void Task_Motor_ControlTick(void);

#endif /* _TASK_MOTOR_H_ */
