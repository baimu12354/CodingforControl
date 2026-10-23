/*********************************************************************************************************************
* 文件名称          NEUQ_Menu
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第一题：单页 LCD 菜单（链表表格菜单）
*
* 结构（和第二题 NEUQ_Menu2.c 是同一套写法：配置表 + 双向循环链表）
*   Menu_Table[]  配置表：一行一个菜单项（名称 / 关联变量 / 最小 / 最大 / 步长；步长 0 = 只读）
*   Menu_Node[]   链表节点：cfg + 行号 + 上次画出的值，prev / next 组成双向循环链表
*   Menu_Build()  上电按配置表建链 —— 以后加一行配置就多一个菜单项，
*                 画行、刷新、光标移动、按键分发这些代码一行都不用改
*
* 页面内容（横屏 320 × 240）
*   标题      LED TIMING  PA8
*   第 1 行   Cur  Plan   当前方案：正在跑的那一套，只读，只在 2 s 周期起点变化
*   第 2 行   Next Plan   待执行方案：" > " 光标所在行，可改，下一周期起点生效
*   提示行    UP/DN:+/-  FWD/BCK:Row
*
* 按键（消抖在 NEUQ_Key.c）
*   PE9 UP / PA0 K0   光标所在项 +步长（1→2→3→4→1 回绕；落在只读行上会自动移到可改行再改）
*   PE7 DOWN          光标所在项 -步长（1→4→3→2→1 回绕）
*   PE8 FWD           光标沿链表移到下一项（循环链表，到头自动绕回）
*   PE10 BACK         光标沿链表移到上一项（循环链表）
*
* 关键点：按键只写"待执行方案"（g_led_plan_pending），绝不碰周期起点、也绝不碰 LED 引脚；
*         LED 时序在 1 ms 中断里跑，刷屏再慢也不会暂停、拉长或重置当前动作。
********************************************************************************************************************/
#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include "stm32f4xx_hal.h"
#include "NEUQ_Key.h"

/* ------------------------- 页面布局（横屏 320 × 240，ASCII_Font24 = 12 × 24） ------------------------- */
#define MENU_CHAR_W         12U      /* 字符宽度(px) */
#define MENU_TITLE_Y        8U       /* 标题 y */
#define MENU_ROW1_Y         56U      /* 第 1 行 y */
#define MENU_ROW_STEP       40U      /* 行间距 */
#define MENU_ROW_BACK_X     0U       /* 行背景左上角 x */
#define MENU_ROW_BACK_Y_OFF 6U       /* 行背景相对文字基线的上移量 */
#define MENU_ROW_W          300U     /* 行背景宽 */
#define MENU_ROW_H          30U      /* 行背景高 */
#define MENU_HINT_Y         208U     /* 提示行 y */

/* ------------------------- 菜单项配置（加一行就多一个菜单项） ------------------------- */
typedef struct
{
    const char       *name;     /* 显示名称（ASCII） */
    volatile int32_t *val;      /* 关联变量地址，如 &g_led_plan_pending */
    int32_t           min;      /* 编辑下限 */
    int32_t           max;      /* 编辑上限（到边界回绕） */
    int32_t           step;     /* 编辑步长；0 = 只读项（如"当前方案"） */
} MenuItemConfig;

/* ------------------------- 链表节点 ------------------------- */
typedef struct MenuNode
{
    const MenuItemConfig *cfg;         /* 指向配置表的某一行 */
    uint8_t               row;         /* 画在第几行（= 配置表下标） */
    int32_t               last_drawn;  /* 上一次画出来的值，值没变就不重画 */
    struct MenuNode      *prev;        /* 双向循环链表：前一项 */
    struct MenuNode      *next;        /* 双向循环链表：后一项 */
} MenuNode;

/* ------------------------------ 接口 ------------------------------ */
void NEUQ_Menu_Init(void);                                      /* 建链 + LED 引擎 + 按键 + LCD + 首屏 */
void NEUQ_Menu_Task(void);                                      /* 主循环任务：沿链表刷新变化过的行 */
void NEUQ_Menu_HandleKey(KeyIdEnum id, KeyEventEnum event);     /* 按键事件 → 移动光标 / 改"待执行方案" */

#endif /* NEUQ_MENU_H_ */
