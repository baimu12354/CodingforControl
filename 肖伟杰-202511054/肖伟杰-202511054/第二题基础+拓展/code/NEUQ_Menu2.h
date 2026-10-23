/*********************************************************************************************************************
* 文件名称          NEUQ_Menu2
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第二题：二级菜单（一级功能选择页 + 三个二级页面）
*
* 结构（配置表 + 双向循环链表，Menu2_Build() 上电自动建链）
*   Menu2Table[]  配置表：名字 / 父页面 / 关联变量地址 / 最小 / 最大 / 步长 / 显示格式
*   Menu2Node[]   链表节点：parent / child 是层级，prev / next 是同级双向循环链表
*                 → 同级上下移动到头会自动绕回，这是链表菜单的特点
*
*   一级 Main Menu
*        ├── 六轴数据显示   AccX..GyrZ：轴名 + 原始计数 / 滤波值 + 单位，六个轴同屏
*        ├── 波形与滤波     Send Axis（选上传轴）/ Send Wave（开关）/ Cutoff（改截止频率）
*        │                  / Save Flash（存 Flash）/ Load Default（恢复默认）
*        └── 姿态解算       Roll / Pitch / Yaw / Update T / Gyro Calib
*
* 按键（消抖在 NEUQ_Key.c）
*     PE9  UP   光标上移 / 编辑时 +1 档         PE7  DOWN 光标下移 / 编辑时 -1 档
*     PE8  FWD  进子页 / 执行动作 / 编辑确认     PE10 BACK  返回上级 / 编辑取消（回滚）
*
* 状态只有一份：选轴、上传开关、截止频率都在 s_set 里，菜单项直接指着它改，
* 所以"页面显示"和"后台实际状态"不可能不一致；切页面只动 selected / page 两个指针。
*
* 本文件还负责两件小事（合进来免得再多两个模块）：
*     ① VOFA+ 上传：FireWater 文本帧，一行两个数值 = 原始值,滤波值
*     ② 参数存 Flash：扇区 5（0x08020000），开头一个 magic 判有效
********************************************************************************************************************/
#ifndef NEUQ_MENU2_H_
#define NEUQ_MENU2_H_

#include "stm32f4xx_hal.h"
#include "NEUQ_Key.h"

/* ------------------------- 页面编号 -------------------------
 * 前 4 个值同时是"页锚点"在配置表 Menu2Table[] 里的下标，顺序不能乱：
 *   PAGE_STR(0)      一级页锚点（哨兵），它的 child 是 [1][2][3] 三个入口
 *   PAGE_ID_AXIS(1)  "6-Axis" 这一项，同时是六轴数据显示页的锚点
 *   PAGE_ID_WAVE(2)  "Wave/Filter" 这一项，同时是波形与滤波设置页的锚点
 *   PAGE_ID_ATT(3)   "Attitude" 这一项，同时是姿态解算页的锚点
 *   PAGE_NUM         哨兵的 parent，表示"没有上级了"
 */
typedef enum
{
    PAGE_STR     = 0,
    PAGE_ID_AXIS = 1,
    PAGE_ID_WAVE = 2,
    PAGE_ID_ATT  = 3,
    PAGE_NUM
} Menu2Page;

/* ------------------------- 数值显示格式 ------------------------- */
typedef enum
{
    FMT_STR = 0,        /* 只读：显示 cfg->str 固定文本 */
    FMT_AXIS,           /* 只读：显示 cfg->val 指向的轴编号对应的轴名（Send Axis 用） */
    FMT_ONOFF,          /* 可编辑：0 = OFF / 1 = ON */
    FMT_HZ,             /* 可编辑：截止频率，值 = 0.1 Hz（100 = 10.0 Hz） */
    FMT_ACCAXIS,        /* 只读：六轴页的加速度行（滤波后 mg） */
    FMT_GYRAXIS,        /* 只读：六轴页的角速度行（滤波后 dps） */
    FMT_ROLL,           /* 只读：横滚 deg */
    FMT_PITCH,          /* 只读：俯仰 deg */
    FMT_YAW,            /* 只读：航向 deg */
    FMT_ACTION          /* 动作项：按 FWD 执行 cfg->act */
} Menu2Fmt;

/* ------------------------- 菜单项配置（加一行就多一个菜单项） ------------------------- */
typedef struct
{
    const char *name;       /* 显示名称（ASCII） */
    Menu2Page   parent;     /* 父页面；一级页填 PAGE_NUM 表示没有父页 */
    int32_t    *val;        /* 关联变量地址；NULL 表示不关联 */
    int32_t     min;        /* 编辑下限（只读项填 0） */
    int32_t     max;        /* 编辑上限 */
    int32_t     step;       /* 编辑步长；0 = 只读/动作项 */
    Menu2Fmt    fmt;        /* 显示格式 */
    const char *str;        /* FMT_STR 的文本 */
    uint8_t     act;        /* FMT_ACTION 的动作编号 */

    /* --- FMT_ACCAXIS / FMT_GYRAXIS 用的轴下标，其他格式忽略 --- */
    uint8_t     axis;
} Menu2ItemConfig;

/* ------------------------- 链表节点 ------------------------- */
typedef struct Menu2Node
{
    const Menu2ItemConfig *cfg;
    struct Menu2Node      *parent;      /* 层级父节点 */
    struct Menu2Node      *child;       /* 层级首个子节点 */
    struct Menu2Node      *prev;        /* 同级前一项（双向循环） */
    struct Menu2Node      *next;        /* 同级后一项 */
} Menu2Node;

/* ------------------------------ 接口 ------------------------------ */
void    NEUQ_Menu2_Init(void);                                   /* LCD + 建链 + 首屏 + 读 Flash */
void    NEUQ_Menu2_ShowMessage(const char *l1, const char *l2);  /* 启动/提示用的临时信息页 */
void    NEUQ_Menu2_Task(void);                                   /* 主循环任务：刷屏 + 上传 */
void    NEUQ_Menu2_HandleKey(KeyIdEnum id, KeyEventEnum event);  /* 按键事件 → 导航/编辑/动作 */
void    NEUQ_Menu2_Redraw(void);                                 /* 立即重画当前页 */

#endif /* NEUQ_MENU2_H_ */
