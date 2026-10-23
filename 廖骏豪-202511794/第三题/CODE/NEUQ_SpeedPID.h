/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_SpeedPID
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
*
* 说明：速度环 PID，控制律：
*         out = Kp·e + Ki·Σe + Kd·(0 − Δ实测) + FF·目标
*       · 积分按"每拍累加"，Ki 的量纲是 输出/RPM/拍（不乘 dt）
*       · 微分用实测变化量，不除 dt、不加低通；Kd 主要用来压过冲
*       · 条件积分抗饱和（不作反算），只正转时输出钳在 [0, +out_max]
*       · 运行中改增益做无扰切换（Ki·Σe 保持不变）
********************************************************************************************************************/
#ifndef _NEUQ_SPEEDPID_H_
#define _NEUQ_SPEEDPID_H_

#include "stm32f4xx_hal.h"

typedef struct
{
    /* ---------------- 生效参数 ---------------- */
    float kp;               /* 比例增益：输出单位 / RPM                        */
    float ki;               /* 积分增益：输出单位 / RPM / 拍                   */
    float kd;               /* 微分增益：输出单位 / RPM / 拍（作用在Δ实测上）  */
    float kff;              /* 速度前馈（可选，默认 0）                        */
    float i_limit;          /* Σe 限幅（量纲 RPM·拍），默认 20000              */
    float slew_limit;       /* 每拍输出最大变化量，<= 0 表示不限幅              */
    float out_min;          /* 输出下限：只正转时 = 0                          */
    float out_max;          /* 输出上限（一般为 9000）                         */

    /* ---------------- 内部状态 ---------------- */
    float   ek_total;       /* Σe：误差累加（RPM·拍）                          */
    float   integral;       /* Ki·Σe：积分项对输出的贡献                       */
    float   d_term;         /* 本拍微分项                                      */
    float   rpm_prev;       /* 上一拍实测转速（微分用）                        */
    float   out_prev;       /* 上一拍输出（变化率限幅用）                      */
    uint8_t started;        /* 首拍标志（第一拍把 rpm_prev 对齐）              */
} NEUQ_Pid_t;

/*  函数简介     PID 初始化（参数取 NEUQ_Config.h 中的默认值）  */
void NEUQ_Pid_Init(NEUQ_Pid_t *p);

/*  函数简介     在线修改增益
 *  备注         积分项对输出的贡献保持不变（无扰切换） */
void NEUQ_Pid_SetGains(NEUQ_Pid_t *p, float kp, float ki, float kd);

/*  函数简介     复位调节器（清积分/微分/输出历史）
 *  备注         启停瞬间调用，保证再次启动从零开始 */
void NEUQ_Pid_Reset(NEUQ_Pid_t *p);

/*  函数简介     PID 定周期运算
 *  参数说明     p 控制器句柄；target 目标(RPM)；meas 实测(RPM)；dt 控制周期(s)（本律不使用）
 *  返回参数     限幅后的输出：只正转时 0 ~ +out_max
 *  备注         无浮点库依赖（只用四则运算），可直接放在定周期中断里 */
float NEUQ_Pid_Update(NEUQ_Pid_t *p, float target, float meas, float dt);

#endif /* _NEUQ_SPEEDPID_H_ */
