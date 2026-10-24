#ifndef _Motorspeed_H_
#define _Motorspeed_H_

#include "NEUQ_lib_include.h"
#include <stdio.h>


/*------------------ 控制周期与换算 ------------------*/
#define SPEED_PERIOD_MS      10                 // PIT 周期（毫秒）
#define SPEED_SCALE          100                // 1000ms / 10ms = 100：计数增量 -> 计数/秒

/*------------------ 限幅常量 ------------------*/
#define INTEGRAL_MAX         1000.0f            // 积分限幅（防积分饱和）
#define PID_OUT_MAX          ((float)PWM_MAX)   // 输出限幅 = 10000

/*------------------ 编码器选择 ------------------*/
#define SPEED_ENC_DEV        Encoder_TIM3
#define SPEED_ENC_PHASE_A    Encoder_TIM3_PhaseA_PA6
#define SPEED_ENC_PHASE_B    Encoder_TIM3_PhaseB_PA7




typedef struct {
    volatile uint8_t  running;      // 1=运行 0=停止
    volatile int32_t  target;       // 目标转速（/秒）
    volatile int32_t  actual;       // 实际转速（/秒）
    volatile float    kp, ki, kd;   // PID 参数：kp比例，ki积分，kd微分
} Speed_Control_t;




void Speed_Init(void);                 // 初始化编码器 + 默认PID参数
void Speed_Start(void);                // 启动闭环（PIT 中断里开始输出）
void Speed_Stop(void);                 // 停止：电机停转，积分清零
void Speed_Target(int32_t target);  // 修改目标转速（运行/停止均可）
void Speed_PID(float kp, float ki, float kd);  // 修改PID（仅停止状态生效）
void SpeedControl_Task_10ms(void);            // PIT 周期任务：测速 + PID + 更新PWM

/* 供协议模块回传使用的只读接口 */
uint8_t Speed_GetRunning(void);
int32_t Speed_GetTarget(void);
int32_t Speed_GetActual(void);
float   Speed_GetKP(void);
float   Speed_GetKI(void);
float   Speed_GetKD(void);

#endif