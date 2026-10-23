/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_Speed
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
*
* 说明：编码器测速层。
*       NEUQ_Encoder 使用 TI12 四倍频，NEUQ_Encoder_Read 返回的是"相对上次清零的计数增量"，
*       所以采用最经典的 M 法测速：每个控制周期(定周期中断)读一次增量并立刻清零，
*       增量 / 每转计数 x 时间系数 = 转速。低速时分辨率由控制周期决定。
********************************************************************************************************************/
#ifndef _NEUQ_SPEED_H_
#define _NEUQ_SPEED_H_

#include "stm32f4xx_hal.h"

/*  函数简介     编码器初始化
 *  使用示例     NEUQ_Speed_Init();
 *  备注         内部会调用 NEUQ_Encoder_init 并把计数清零、换算系数算好            */
void NEUQ_Speed_Init(void);

/*  函数简介     定周期测速（在控制中断里调用，每个周期必须调用一次）
 *  返回参数     本次测得的转速，单位 RPM（减速箱输出轴），已完成方向修正与低通滤波
 *  使用示例     float speed = NEUQ_Speed_Update();
 *  备注         函数内部读增量 + 清零计数，因此不能在一个周期里调用多次（第二次会是 0） */
float NEUQ_Speed_Update(void);

/*  函数简介     读取最近一次（滤波后）转速，不触发新的采样
 *  返回参数     RPM                                                              */
float NEUQ_Speed_Get(void);

/*  函数简介     读取最近一次编码器原始计数增量（未换算）
 *  返回参数     计数增量（带符号）                                               */
int32_t NEUQ_Speed_GetRawDelta(void);

/*  函数简介     读取换算系数：1 个编码器计数对应多少 RPM
 *  返回参数     RPM / count                                                      */
float NEUQ_Speed_GetRpmPerCount(void);

/*  函数简介     运行时修改测速方向符号（+1 或 -1）
 *  参数说明     sign   >0 取 +1，<0 取 -1，其它按 +1 处理
 *  使用示例     NEUQ_Speed_SetDirSign(-1.0f);
 *  备注         对应串口命令 0x14：现场判断有没有"正反馈飞车"时，不用重新烧程序，
 *               发一条 CC DD 14 04 00 00 80 3F / 80 BF 就能把符号翻过来试      */
void NEUQ_Speed_SetDirSign(float sign);

/*  函数简介     读取当前测速方向符号（+1 或 -1）
 *  返回参数     +1.0f / -1.0f                                                     */
float NEUQ_Speed_GetDirSign(void);

#endif /* _NEUQ_SPEED_H_ */
