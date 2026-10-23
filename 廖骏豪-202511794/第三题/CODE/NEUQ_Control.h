/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_Control
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
*
* 说明：闭环控制层，也是"通信"与"控制"之间的隔离层。
*       数据流：
*         主循环(通信解析) --写指令邮箱--> 定周期中断(Control_ISR) --读快照--> 主循环(回传)
*       两个方向都不允许对方阻塞自己：
*         · 通信慢 / 断线，只影响指令更新，控制节拍照常跑；
*         · 中断里只做读编码器 + PID + 写 PWM，不等待任何标志，耗时固定。
********************************************************************************************************************/
#ifndef _NEUQ_CONTROL_H_
#define _NEUQ_CONTROL_H_

#include "stm32f4xx_hal.h"

/* 回传给上位机的数据快照：全部是"当前实际生效"的值 */
typedef struct
{
    float    target;    /* 当前生效目标转速(RPM)  */
    float    actual;    /* 实际转速(RPM，已滤波)  */
    float    kp;        /* 当前生效 Kp            */
    float    ki;        /* 当前生效 Ki            */
    float    kd;        /* 当前生效 Kd            */
    float    output;    /* 当前输出(-10000~10000) */
    float    kff;       /* 当前生效前馈增益       */
    float    slew;      /* 当前生效输出变化率限幅 */
    uint8_t  enable;    /* 1 = 闭环运行中         */
    uint8_t  openloop;  /* 1 = 开环调试模式       */
    uint32_t tick;      /* 控制节拍计数           */
} NEUQ_Snapshot;

/*  函数简介     闭环控制初始化（初始化 PID、指令邮箱，上电默认"停止等待启动"） */
void NEUQ_Control_Init(void);

/*  函数简介     定周期控制中断体：读编码器 -> 换算转速 -> PID -> 更新 PWM -> 更新快照
 *  备注         由 NEUQ_PIT_1_CALLBACK(TIM6 中断) 调用，周期 = NEUQ_CONTROL_PERIOD_MS */
void NEUQ_Control_ISR(void);

/*  函数简介     取回传快照（主循环调用）
 *  参数说明     dst   输出的快照结构体指针
 *  备注         内部短暂关中断拷贝，保证 6 个通道来自同一个控制周期，不会出现"新目标配旧转速" */
void NEUQ_Control_GetSnapshot(NEUQ_Snapshot *dst);

/*  函数简介     取当前控制节拍计数（主循环用于回传节拍与帧超时判断） */
uint32_t NEUQ_Control_GetTick(void);

/*  函数简介     取当前运行状态（0 = 已停止 / 未启动，1 = 闭环运行中）
 *  备注         通信模块用它判断"PID 参数是否允许修改"（题目要求：仅停止时允许） */
uint8_t NEUQ_Control_GetState(void);

/*=========================== 以下为通信模块使用的指令接口（主循环上下文调用） ===========================*/
/*  说明：这些函数只往指令邮箱写"期望状态"，由控制中断在下一个节拍统一应用，
 *        因此不会出现参数更新到一半被中断取走的情况。                          */

void NEUQ_Control_ReqStart(void);                    /* 启动闭环                          */
void NEUQ_Control_ReqStop(void);                     /* 停止闭环，输出清零（按配置刹车）  */
void NEUQ_Control_ReqTarget(float target);           /* 修改目标转速(RPM)，自动限幅       */
void NEUQ_Control_ReqPid(float kp, float ki, float kd);   /* 修改 PID 参数（一帧三个）     */
void NEUQ_Control_ReqPidOne(uint8_t sel, float val);  /* 单参数修改 PID：1=Kp 2=Ki 3=Kd，
                                                       * 专给 VOFA+ 滑条用（%% 一次只能替换一个 float） */
void NEUQ_Control_ReqGain2(float kff, float slew);   /* 修改前馈增益/输出变化率限幅        */
void NEUQ_Control_ReqClear(void);                    /* 清 PID 积分与内部状态              */
void NEUQ_Control_ReqOpenloop(float duty);           /* 开环调试：直接给占空比（仅停止时有效） */

#endif /* _NEUQ_CONTROL_H_ */
