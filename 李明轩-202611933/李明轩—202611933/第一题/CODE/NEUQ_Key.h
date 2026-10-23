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
* 文件名称          NEUQ_Key
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#ifndef __NEUQ_KEY_H__
#define __NEUQ_KEY_H__

#include "stm32f4xx_hal.h"
#include "NEUQ_LED.h"   // 【连接点② 接口引入】引入 LED 程序声明的 4 个方案函数（见 NEUQ_LED.h【连接点①】）

/*
 * 菜单节点：用「双向循环链表」组织菜单选项
 * 每个节点就是菜单里的一个选项（light1 ~ light4）
 */
typedef struct Menu_Node {    //链表结构
    char     name[16];        // 方案名，例如 "light1"
    uint8_t  id;              // 方案编号 1 ~ 4
    void   (*scheme)(void);   // 【连接点② 函数指针】保存该选项对应的 LED 动作函数，执行时经它调用（见⑤）
    struct Menu_Node *prev;   // 指向前一个节点
    struct Menu_Node *next;   // 指向后一个节点
} Menu_Node;

void Menu_Init(void);   // 初始化菜单：LCD、按键、ADC、链表
void Menu_Task(void);   // 菜单主任务，放在 while(1) 中反复调用

#endif  /* __NEUQ_KEY_H__ */
