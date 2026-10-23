/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_LED
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2026-10-01        ——                  四套 LED 显示方案声明
********************************************************************************************************************/
#ifndef __NEUQ_LED_H__
#define __NEUQ_LED_H__

#include "stm32f4xx_hal.h"

/* ===== 【连接点① 接口声明】LED 程序对外提供的 4 个函数，菜单程序靠它们调用 LED =====
 * 这些函数在 NEUQ_LED.c 中实现（见【连接点⑥ 实现】），
 * 被 NEUQ_Key.h 引入（见【连接点②】），最终在 NEUQ_Key.c 中绑定并执行（见③④⑤）。
 * 每个方案只点亮一个对应的 LED，其余 LED 保持熄灭。
 */
void LED_Scheme_Light1(void);   // 方案1：仅 LED1 闪
void LED_Scheme_Light2(void);   // 方案2：仅 LED2 闪
void LED_Scheme_Light3(void);   // 方案3：仅 LED3 闪
void LED_Scheme_Light4(void);   // 方案4：仅 LED4 闪

void LED_Init(void);            // 初始化 LED 引脚（在 User_Main 中调用）

#endif  /* __NEUQ_LED_H__ */
