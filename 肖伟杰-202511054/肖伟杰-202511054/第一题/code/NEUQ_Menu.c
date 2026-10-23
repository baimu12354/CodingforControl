/*********************************************************************************************************************
* 文件名称          NEUQ_Menu
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第一题：单页 LCD 菜单（链表表格菜单：配置表 + 双向循环链表）
*
* 数据流
*   Menu_Table[]（配置表）
*        │ Menu_Build() 上电建链
*        ▼
*   Menu_Node[] ←prev/next→ 双向循环链表        Menu_Sel = 光标所在节点
*        │
*        ├─ Menu_RefreshList()  沿链表走一圈，值变了的行才重画（主循环里调用）
*        ├─ Menu_MoveCursor()   光标沿链表前后移动（FWD / BACK）
*        └─ Menu_Adjust()       光标项 +/- 一个步长，到边界回绕（UP / DOWN / K0）
*
* 关键点：整个过程只写 g_led_plan_pending，绝不碰周期起点、也绝不碰 LED 引脚，
*         所以连续按键、长按、刷屏都不会暂停/拉长/重置正在跑的 LED 动作。
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "NEUQ_LedPlan.h"
#include "NEUQ_Key.h"
#include "NEUQ_Menu.h"

/* ------------------------- 页面文字 ------------------------- */
#define MENU_TITLE_TEXT   "LED TIMING  PA8"
#define MENU_HINT_TEXT    "UP/DN:+/-  FWD/BCK:Row"

#define MENU_HL_BACK      LCD_BLUE      /* 光标行的底色（高亮） */
#define MENU_HL_PEN       LCD_WHITE
#define MENU_LABEL_X      20U           /* 标签起点 x */
#define MENU_VALUE_X      140U          /* 数值起点 x */
#define MENU_ROW_X        4U            /* 光标 ">" 的 x */

/* ------------------------- 菜单配置表（加一行就多一个菜单项） ------------------------- */
static const MenuItemConfig Menu_Table[] =
{
    /* 名称          关联变量              最小 最大 步长 */
    { "Cur  Plan",  &g_led_plan_active,     1,   4,   0 },   /* 只读：正在跑的那一套 */
    { "Next Plan",  &g_led_plan_pending,    1,   4,   1 },   /* 可改：下一周期起点生效 */
};

#define MENU_ITEM_NUM   (sizeof(Menu_Table) / sizeof(Menu_Table[0]))

/* ------------------------- 链表 ------------------------- */
static MenuNode  Menu_Node[MENU_ITEM_NUM];      /* 节点存储，和配置表一一对应 */
static MenuNode *Menu_Head = NULL;              /* 链表头（第 1 行那个节点） */
static MenuNode *Menu_Sel  = NULL;              /* 光标所在节点 */

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     读一个菜单项的当前值（显示和编辑都从这里取，页面显示和真实状态不会不一致）
*------------------------------------------------------------------------------------------------------------------*/
static int32_t Menu_ReadValue(const MenuNode *node)
{
    return *(node->cfg->val);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     画一行：整行清底 → 标签 → ':' → 数值；highlight=1 时前面画 ">" 光标
*------------------------------------------------------------------------------------------------------------------*/
static void Menu_DrawRow(const MenuNode *node, uint8_t highlight)
{
    const MenuItemConfig *cfg = node->cfg;
    uint16_t y      = (uint16_t)(MENU_ROW1_Y + (uint16_t)MENU_ROW_STEP * node->row);
    uint16_t back_y = (uint16_t)(y - MENU_ROW_BACK_Y_OFF);

    NEUQ_LCD200_SetBackColor(highlight ? MENU_HL_BACK : LCD_WHITE);
    NEUQ_LCD200_SetPenColor(highlight ? MENU_HL_PEN : LCD_BLACK);

    /* 整行先清一次底：旧数值比新数值长时不会留残影，去掉光标时也一并擦干净 */
    NEUQ_LCD200_ClearRect(MENU_ROW_BACK_X, back_y, MENU_ROW_W, MENU_ROW_H);

    if (highlight != 0U)
    {
        NEUQ_LCD200_DisplayString(MENU_ROW_X, y, (char *)">");     /* 光标：这一行是当前项 */
    }

    NEUQ_LCD200_DisplayString(MENU_LABEL_X, y, (char *)cfg->name);
    NEUQ_LCD200_DisplayChar((uint16_t)(MENU_VALUE_X - MENU_CHAR_W), y, ':');
    NEUQ_LCD200_Display_int32(MENU_VALUE_X, y, Menu_ReadValue(node), 6U);

    NEUQ_LCD200_SetBackColor(LCD_WHITE);        /* 恢复默认配色，不影响后面的绘制 */
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     沿循环链表走一圈，把值变了的项重画（表格驱动：新增菜单项不用改这里）
* 参数说明     force：1 = 全部重画（首屏用）
*------------------------------------------------------------------------------------------------------------------*/
static void Menu_RefreshList(uint8_t force)
{
    MenuNode *node;

    if (Menu_Head == NULL)
    {
        return;
    }

    node = Menu_Head;
    do
    {
        int32_t value = Menu_ReadValue(node);

        if ((force != 0U) || (value != node->last_drawn))
        {
            node->last_drawn = value;
            Menu_DrawRow(node, (uint8_t)((node == Menu_Sel) ? 1U : 0U));
        }

        node = node->next;                      /* 循环链表：走到头自然回到链表头 */
    } while (node != Menu_Head);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     从 from 开始沿链表找第一个可编辑项（步长非 0）
* 返回参数     找不到返回 NULL
*------------------------------------------------------------------------------------------------------------------*/
static MenuNode *Menu_FindEditable(MenuNode *from)
{
    MenuNode *node = from;
    uint8_t   i;

    for (i = 0U; i < (uint8_t)MENU_ITEM_NUM; i++)
    {
        if (node->cfg->step != 0)
        {
            return node;
        }
        node = node->next;
    }
    return NULL;                                /* 整页都是只读项（本页不会出现） */
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     光标沿链表移到下一项 / 上一项（循环链表：到头自动绕回）
*------------------------------------------------------------------------------------------------------------------*/
static void Menu_MoveCursor(int32_t dir)
{
    MenuNode *old;

    if ((Menu_Head == NULL) || (Menu_Sel == NULL))
    {
        return;
    }

    old      = Menu_Sel;
    Menu_Sel = (dir > 0) ? Menu_Sel->next : Menu_Sel->prev;

    Menu_DrawRow(old, 0U);                      /* 旧行去掉光标 */
    Menu_DrawRow(Menu_Sel, 1U);                 /* 新行画上光标 */
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     光标所在项 +/- 一个步长，到边界回绕：1→2→3→4→1
* 备注信息     光标若落在只读项（当前方案）上，会自动挪到最近的可改项再改它，
*              这样在只读行上按键也不会像死机一样没反应。
*------------------------------------------------------------------------------------------------------------------*/
static void Menu_Adjust(int32_t dir)
{
    MenuNode *node;
    int32_t   value;

    node = Menu_FindEditable(Menu_Sel);
    if (node == NULL)
    {
        return;
    }

    if (node != Menu_Sel)                       /* 从只读行挪到可改行：光标跟着走 */
    {
        Menu_DrawRow(Menu_Sel, 0U);
        Menu_Sel = node;
        Menu_DrawRow(Menu_Sel, 1U);
    }

    value = Menu_ReadValue(node) + dir * node->cfg->step;

    if (value > node->cfg->max)
    {
        value = node->cfg->min;                 /* 到上限回到下限，方便连续按键依次展示四套动作 */
    }
    if (value < node->cfg->min)
    {
        value = node->cfg->max;
    }

    *(node->cfg->val) = value;                  /* ★ 只写"待执行方案"，别的什么都不动 */

    node->last_drawn = value;                   /* 立刻刷新这一行，按键马上能看到变化 */
    Menu_DrawRow(node, 1U);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     按配置表建链：prev / next 首尾相接，形成双向循环链表
*------------------------------------------------------------------------------------------------------------------*/
static void Menu_Build(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)MENU_ITEM_NUM; i++)
    {
        Menu_Node[i].cfg        = &Menu_Table[i];
        Menu_Node[i].row        = i;
        Menu_Node[i].last_drawn = 0x7FFFFFFF;   /* 不可能等于真实值，保证首屏一定重画 */
        Menu_Node[i].prev       = &Menu_Node[(uint8_t)((i + (uint8_t)MENU_ITEM_NUM - 1U) % (uint8_t)MENU_ITEM_NUM)];
        Menu_Node[i].next       = &Menu_Node[(uint8_t)((i + 1U) % (uint8_t)MENU_ITEM_NUM)];
    }

    Menu_Head = &Menu_Node[0];
    Menu_Sel  = Menu_FindEditable(Menu_Head);   /* 光标默认停在可改的"待执行方案"上 */
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     菜单初始化：建链 → LED 引擎 → 按键 → LCD → 首屏
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Menu_Init(void)
{
    Menu_Build();                               /* ① 先建链，后面画的都是链表里的节点 */

    NEUQ_LedPlan_Init();                        /* ② PA8 + 时序引擎：上电即从方案 1 开始循环 */
    NEUQ_Key_Init();                            /* ③ 5 个按键配成上拉输入 */

    NEUQ_LCD200_init();                         /* ④ 屏幕初始化（库内部最后会把背光拉高） */
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross); /*    横屏：320 宽 × 240 高 */
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font24);    /*    12 × 24 字体 */

    NEUQ_LCD200_SetBackColor(LCD_WHITE);        /* ⑤ 首屏：整屏清一次，再画标题和提示行 */
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(70U, MENU_TITLE_Y, (char *)MENU_TITLE_TEXT);
    NEUQ_LCD200_DisplayString(8U, MENU_HINT_Y, (char *)MENU_HINT_TEXT);

    Menu_RefreshList(1U);                       /* ⑥ 沿链表把每一项都画一遍 */
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     菜单任务（主循环里循环调用）：沿链表刷新变化过的行
* 备注信息     "当前方案"只在 2 s 周期边界变，所以这一行也是到边界才重画；
*              LED 时序在 1 ms 节拍里跑，这里刷屏再慢也不会影响 LED 动作。
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Menu_Task(void)
{
    Menu_RefreshList(0U);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     按键处理：按链表里"光标所在项"的类型分发
* 参数说明     id：按键编号；event：SHORT / LONG（长按由 NEUQ_Key.c 按节奏重复给出）
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Menu_HandleKey(KeyIdEnum id, KeyEventEnum event)
{
    if (event == KEY_EVENT_NONE)
    {
        return;
    }

    switch (id)
    {
        case KEY_ID_UP:                         /* 光标项 +1（只读行会自动移到可改行） */
        case KEY_ID_K0:
            Menu_Adjust(+1);
            break;

        case KEY_ID_DOWN:                       /* 光标项 -1 */
            Menu_Adjust(-1);
            break;

        case KEY_ID_FORWARD:                    /* 光标沿链表后移一项 */
            Menu_MoveCursor(+1);
            break;

        case KEY_ID_BACK:                       /* 光标沿链表前移一项 */
            Menu_MoveCursor(-1);
            break;

        default:
            break;
    }
}
