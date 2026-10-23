/*********************************************************************************************************************
* 文件名称          NEUQ_Menu2
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第二题：二级菜单（配置表 + 双向循环链表）实现
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "NEUQ_IMU.h"
#include "NEUQ_Attitude.h"
#include "NEUQ_Menu2.h"

/* VOFA+ 串口：1 = USART1(PA9，ST-Link 虚拟串口)；2 = USART3(PB10，需外接 USB-TTL)。 */
#define VOFA_UART_NO        1

#if (VOFA_UART_NO == 1)
#  define VOFA_UART         (UART_0)        /* USART1：PA9(TX) / PA10(RX)，ST-Link 虚拟串口 */
#elif (VOFA_UART_NO == 2)
#  define VOFA_UART         (UART_2)        /* USART3：PB10(TX) / PB11(RX)，需要 USB-TTL */
#else
#  error "VOFA_UART_NO 只能是 1（USART1/PA9）或 2（USART3/PB10）"
#endif

/* NEUQ_Uart_init / NEUQ_Uart_write_data 的声明在 Library 的 NEUQ_Uart.h 里。
 * 这里再写一遍是为了让"本文件用到的是哪一路串口"一眼可见（两个声明必须完全一致）。 */
void NEUQ_Uart_init(UART_DEV_ENUM DEV_select, UART_BAUD_ENUM BAUD_select);
void NEUQ_Uart_write_data(UART_DEV_ENUM DEV_select, uint8_t *pdata, uint32_t len);

/* ------------------------- 屏显布局 -------------------------
 * 横屏 320×240，ASCII_Font24 的字宽 12、字高 24：每行 26 个字符。 */
#define ROW_Y0      24                      /* 第 1 行 y 坐标 */
#define ROW_H       24                      /* 行距 = 字高 */
#define CHAR_W      12                      /* 字宽 */
#define CELL_ROWS   10                      /* 屏幕最多 10 行 */
#define CELL_COLS   26                      /* 每行最多 26 个字符 */

/* 每行上一次显示的内容与高亮状态：只在内容或配色变化时写屏，避免整屏闪。 */
static char    s_shown[CELL_ROWS][CELL_COLS];
static uint8_t s_shown_hl[CELL_ROWS];

/* ------------------------- 动作编号 ------------------------- */
#define ACT_SAVE    1
#define ACT_DEFAULT 2
#define ACT_CALIB   3

/* ------------------------- 可保存的参数（全工程只有这一份） ------------------------- */
static int32_t s_axis    = AXIS_ACC_X;      /* 上传轴 */
static int32_t s_send    = 0;               /* 上传开关 */
static int32_t s_cutoff  = 100;             /* 截止频率，单位 0.1Hz（100 = 10.0Hz） */

/* 需要整屏清一次（上电首屏 / 换页 / 刚从临时信息页回来）。 */
static uint8_t s_need_clear = 1U;

/* 配置表：name / parent / val / min / max / step / fmt / str / act / axis。
 * [0] 是哨兵；[1][2][3] 既是主菜单入口，也是各自页面的锚点（parent = PAGE_STR）。 */
static const Menu2ItemConfig Menu2Table[] =
{
    /* ========== [0] 一级页锚点（哨兵） ========== */
    { "Main", PAGE_NUM, NULL, 0, 0, 0, FMT_STR, NULL, 0, 0 },

    /* ========== [1][2][3] 三个入口，同时是各自页面的锚点 ==========
     * str 用来显示右侧说明；" Wave" / " R/P/Y" 自带前导空格，
     * 是因为 %4s 对超过 4 字符的串不补空格，短名("6-Axis")靠它对齐、长名靠它隔开。 */
    { "6-Axis",      PAGE_STR, NULL, 0, 0, 0, FMT_STR, "Acc+Gyro", 0, 0 },
    { "Wave/Filter", PAGE_STR, NULL, 0, 0, 0, FMT_STR, " to VOFA+", 0, 0 },
    { "Attitude",    PAGE_STR, NULL, 0, 0, 0, FMT_STR, " R/P/Y",   0, 0 },

    /* ========== [4..9] 六轴数据显示页 ========== */
    { "AccX", PAGE_ID_AXIS, NULL, 0, 0, 0, FMT_ACCAXIS, NULL, 0, AXIS_ACC_X },
    { "AccY", PAGE_ID_AXIS, NULL, 0, 0, 0, FMT_ACCAXIS, NULL, 0, AXIS_ACC_Y },
    { "AccZ", PAGE_ID_AXIS, NULL, 0, 0, 0, FMT_ACCAXIS, NULL, 0, AXIS_ACC_Z },
    { "GyrX", PAGE_ID_AXIS, NULL, 0, 0, 0, FMT_GYRAXIS, NULL, 0, AXIS_GYR_X },
    { "GyrY", PAGE_ID_AXIS, NULL, 0, 0, 0, FMT_GYRAXIS, NULL, 0, AXIS_GYR_Y },
    { "GyrZ", PAGE_ID_AXIS, NULL, 0, 0, 0, FMT_GYRAXIS, NULL, 0, AXIS_GYR_Z },

    /* ========== [10..14] 波形与滤波设置页 ========== */
    { "Send Axis",  PAGE_ID_WAVE, &s_axis,   0, AXIS_NUM - 1, 1, FMT_AXIS,  NULL,    0, 0 },
    { "Send Wave",  PAGE_ID_WAVE, &s_send,   0, 1,            1, FMT_ONOFF, NULL,    0, 0 },
    { "Cutoff",     PAGE_ID_WAVE, &s_cutoff, 10, 1000,        1, FMT_HZ,    NULL,    0, 0 },
    { "Save Flash", PAGE_ID_WAVE, NULL, 0, 0, 0, FMT_ACTION, "PRESS", ACT_SAVE,    0 },
    { "Load Deflt", PAGE_ID_WAVE, NULL, 0, 0, 0, FMT_ACTION, "PRESS", ACT_DEFAULT, 0 },

    /* ========== [15..19] 姿态解算页 ========== */
    { "Roll",       PAGE_ID_ATT, NULL, 0, 0, 0, FMT_ROLL,  "deg", 0, 0 },
    { "Pitch",      PAGE_ID_ATT, NULL, 0, 0, 0, FMT_PITCH, "deg", 0, 0 },
    { "Yaw",        PAGE_ID_ATT, NULL, 0, 0, 0, FMT_YAW,   "deg", 0, 0 },
    { "Update T",   PAGE_ID_ATT, NULL, 0, 0, 0, FMT_STR,   "5 ms", 0, 0 },
    { "Gyro Calib", PAGE_ID_ATT, NULL, 0, 0, 0, FMT_ACTION, "PRESS", ACT_CALIB, 0 },
};

/* 页锚点编号就是 Menu2Table 的下标，直接用 NEUQ_Menu2.h 里的 PAGE_STR / PAGE_ID_* 枚举。
 * 注意 [1][2][3] 既是"页锚点"，又是主菜单里的三个入口项，一套节点两个身份。 */

#define NODE_NUM        (sizeof(Menu2Table) / sizeof(Menu2Table[0]))

static const char *const AXIS_NAME[AXIS_NUM] = { "AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ" };

/* ------------------------- 链表节点 + 当前状态 ------------------------- */
static Menu2Node *s_nodes;                  /* 节点数组（Init 里建） */
static Menu2Node *s_page;                   /* 当前页面 */
static Menu2Node *s_sel;                    /* 当前高亮项 */
static uint8_t    s_edit = 0;               /* 1 = 正在编辑参数 */
static int32_t    s_old  = 0;               /* 进入编辑前的原值（BACK 回滚用） */
static uint32_t   s_draw = 0;               /* 上次刷屏时刻 */

/* ------------------------- Flash 参数区 ------------------------- */
#define FLASH_ADDR      0x08020000U         /* 扇区 5（ROM 只用到 0x08020000 之前） */
#define FLASH_MAGIC     0x3632514EU         /* "NQ26"：格式变了就换 magic，旧记录自动作废 */

typedef struct
{
    uint32_t magic;
    int32_t  axis;
    int32_t  send;
    int32_t  cutoff;
} FlashSet;

/*===================================================================================================================
*                                          一、链表：建链与查找
*==================================================================================================================*/

/* 把 node 挂到 host 的子链末尾（host->child->prev 就是环尾）。 */
static void Menu2_AppendChild(Menu2Node *host, Menu2Node *node)
{
    Menu2Node *last;

    node->parent = host;

    if (host->child == NULL)
    {
        host->child = node;                 /* 第一个子节点 */
        node->next   = node;                /* 自成单节点环，供后继节点当"环尾"用 */
        node->prev   = node;
        return;
    }

    last = host->child->prev;               /* 环尾 */
    last->next          = node;
    node->prev          = last;
    node->next          = host->child;
    host->child->prev   = node;
}

/* 建链：每个节点挂到 parent 指向的节点的子链末尾；parent = PAGE_NUM 的是哨兵。 */
static void Menu2_Build(void)
{
    uint8_t i;

    for (i = 0; i < NODE_NUM; i++)
    {
        s_nodes[i].cfg    = &Menu2Table[i];
        s_nodes[i].parent = NULL;
        s_nodes[i].child  = NULL;
        s_nodes[i].prev   = &s_nodes[i];
        s_nodes[i].next   = &s_nodes[i];
    }

    for (i = 0; i < NODE_NUM; i++)
    {
        uint8_t host = (uint8_t)Menu2Table[i].parent;

        /* 只有哨兵 [0] 的 parent 是 PAGE_NUM。必须按枚举值判断，
         * 不能写 host >= NODE_NUM：PAGE_NUM = 5，而节点总数是 20，
         * 那样哨兵会被当成 "AccY 的子节点" 挂进六轴页，导致
         * ① 光标停在 AccY 按 FWD 会进入一个标题为 "?" 的假页面；
         * ② 在主菜单按 BACK 会退到那个假页面，再也回不到一级页。 */
        if ((Menu2Table[i].parent == PAGE_NUM) || (host >= NODE_NUM))
        {
            continue;                       /* 哨兵没有父项；越界值也直接忽略 */
        }
        Menu2_AppendChild(&s_nodes[host], &s_nodes[i]);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     同级上/下移动光标（双向循环链表，到头自动绕回）
* 备注信息     UP = next（向前），DOWN = prev（向后）。环上没有需要跳过的节点，
*              因为三个入口本身就是页锚点，不额外造哑节点。
*------------------------------------------------------------------------------------------------------------------*/
static void Menu2_Move(int dir)
{
    if (s_sel == NULL)
    {
        return;
    }
    s_sel = (dir > 0) ? s_sel->next : s_sel->prev;
}

/*===================================================================================================================
*                                          二、参数：Flash 读写
*==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     恢复默认值（无有效 Flash 记录时也走这里）
*------------------------------------------------------------------------------------------------------------------*/
static void Menu2_Default(void)
{
    s_axis   = AXIS_ACC_X;
    s_send   = 0;
    s_cutoff = 100;                     /* 10.0 Hz：既压得住快速抖动，也跟得上手动倾斜 */
    NEUQ_IMU_SetFc((uint16_t)s_cutoff);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     把当前参数写进 Flash 扇区 5
* 返 回 值     1 = 成功
*------------------------------------------------------------------------------------------------------------------*/
static uint8_t Menu2_Save(void)
{
    FlashSet              set;
    FLASH_EraseInitTypeDef erase;
    uint32_t              err = 0;
    const uint32_t       *src;
    uint32_t              addr;
    uint8_t               i;
    uint8_t               ok = 1;

    set.magic  = FLASH_MAGIC;
    set.axis   = s_axis;
    set.send   = s_send;
    set.cutoff = s_cutoff;

    HAL_FLASH_Unlock();

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector       = FLASH_SECTOR_5;
    erase.NbSectors    = 1;
    if (HAL_FLASHEx_Erase(&erase, &err) != HAL_OK)
    {
        ok = 0;
    }

    src  = (const uint32_t *)&set;
    addr = FLASH_ADDR;
    for (i = 0; ok != 0U && i < (sizeof(FlashSet) / 4U); i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t)src[i]) != HAL_OK)
        {
            ok = 0;
        }
        addr += 4U;
    }

    HAL_FLASH_Lock();

    return ok;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     上电读 Flash：magic 对就用记录里的值，否则用默认值
*------------------------------------------------------------------------------------------------------------------*/
static void Menu2_Load(void)
{
    const FlashSet *rec = (const FlashSet *)FLASH_ADDR;

    if (rec->magic == FLASH_MAGIC &&
        rec->axis >= 0 && rec->axis < AXIS_NUM &&
        rec->cutoff >= 10 && rec->cutoff <= 1000)
    {
        s_axis   = rec->axis;
        s_send   = (rec->send != 0) ? 1 : 0;
        s_cutoff = rec->cutoff;
    }
    else
    {
        Menu2_Default();
    }

    NEUQ_IMU_SetFc((uint16_t)s_cutoff);
}

/*===================================================================================================================
*                                          三、画页面
*==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     画一行（行级缓存：内容没变就不写屏）
* 输入参数     row : 行号（0 = 标题）；text : 内容；highlight : 1 = 蓝底白字
*
* 高亮变化时整行重画（否则旧行尾的 '>' 会残留）；内容变化时只重画变化的字符范围。
* 这样既不会整屏闪，也不需要维护 10×26 的逐格缓存。
*------------------------------------------------------------------------------------------------------------------*/
static void Menu2_Row(uint8_t row, const char *text, uint8_t highlight)
{
    uint16_t y;
    uint8_t  i;
    uint8_t  len;
    uint8_t  first   = 0U;
    uint8_t  last    = 0U;
    uint8_t  changed = 0U;
    uint8_t  c;

    if ((row >= CELL_ROWS) || (text == NULL))
    {
        return;
    }

    y   = (uint16_t)(ROW_Y0 + row * ROW_H);
    len = (uint8_t)strlen(text);
    if (len > CELL_COLS)
    {
        len = CELL_COLS;
    }

    if (s_shown_hl[row] != highlight)
    {
        first   = 0U;
        last    = CELL_COLS - 1U;
        changed = 1U;
        s_shown_hl[row] = highlight;
    }
    else
    {
        for (i = 0U; i < CELL_COLS; i++)
        {
            c = (i < len) ? (uint8_t)text[i] : (uint8_t)' ';
            if ((uint8_t)s_shown[row][i] != c)
            {
                if (changed == 0U)
                {
                    first   = i;
                    changed = 1U;
                }
                last = i;
            }
        }
    }

    if (changed == 0U)
    {
        return;
    }

    if (highlight != 0U)
    {
        NEUQ_LCD200_SetBackColor(LCD_BLUE);
        NEUQ_LCD200_SetPenColor(LCD_WHITE);
    }
    else
    {
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
    }

    for (i = first; i <= last; i++)
    {
        c = (i < len) ? (uint8_t)text[i] : (uint8_t)' ';
        NEUQ_LCD200_DisplayChar((uint16_t)((uint16_t)i * CHAR_W), y, c);
        s_shown[row][i] = (char)c;
    }

    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     作废行缓存：换页/整屏清/临时信息页之后调用，下次重画会整行重写
*------------------------------------------------------------------------------------------------------------------*/
static void Menu2_CellsInvalidate(void)
{
    memset(s_shown, 0xFF, sizeof(s_shown));
    memset(s_shown_hl, 0xFF, sizeof(s_shown_hl));
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     把"unit 分之一为单位"的定点整数拼成带一位小数的文本
* 输入参数     dst : 目标缓冲（至少 12 字节），v : 数值
*              unit : 1 = mg，10 = 0.01dps，100 = 0.01deg
* 备注信息     手写而不用 %f：AC5 + microLIB 下浮点格式化会拖进一大堆代码。
*              取第一位小数必须写 (a % unit) * 10 / unit，
*              不能写 (a % unit) / (unit / 10) —— unit = 1 时 unit/10 == 0，会整数除零！
*------------------------------------------------------------------------------------------------------------------*/
static void Menu2_Fixed1(char *dst, int32_t v, int32_t unit)
{
    int32_t a = (v < 0) ? -v : v;
    int32_t i = a / unit;
    int32_t d = (a % unit) * 10 / unit;     /* 第一位小数 */

    sprintf(dst, "%s%ld.%ld", (v < 0) ? "-" : "", (long)i, (long)d);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     按配置表把某一项画成一行
* 输入参数     row : 行号，cfg : 该项配置
* 备注信息     高亮项用 '>' 标在行首，编辑中用 '<' 标在行尾
*------------------------------------------------------------------------------------------------------------------*/
static void Menu2_DrawItem(uint8_t row, const Menu2ItemConfig *cfg)
{
    char    line[40];
    char    end[4];
    uint8_t selected = (uint8_t)((cfg == s_sel->cfg) ? 1U : 0U);

    end[0] = '\0';
    if (selected != 0U)
    {
        strcpy(end, s_edit ? " <" : " >");      /* 编辑中行尾显示 '<'，平时显示 '>' */
    }

    switch (cfg->fmt)
    {
        /* ---- 六轴页：物理值 + 单位（行宽最多 23 字符） ---- */
        case FMT_ACCAXIS:
        case FMT_GYRAXIS:
        {
            char phys[12];
            char unit[4];
            int  is_acc = (cfg->fmt == FMT_ACCAXIS) ? 1 : 0;

            if (is_acc != 0)
            {
                Menu2_Fixed1(phys, (int32_t)imu_mg[cfg->axis - AXIS_ACC_X], 1);
                strcpy(unit, "mg");
            }
            else
            {
                Menu2_Fixed1(phys, (int32_t)imu_cdps[cfg->axis - AXIS_GYR_X], 10);
                strcpy(unit, "dps");
            }

            sprintf(line, "%s:%8s%s%s", cfg->name, phys, unit, end);
            break;
        }

        /* ---- 参数页 ----
         * 名称列 11 位左对齐 + 数值列 4 位右对齐 + 光标 2 位 = 17，最长也留有余量：
         *   "Cutoff       10.0Hz >"  = 21
         *   "Send Wave      OFF >"   = 21
         *   "Save Flash    PRESS >"  = 21
         *   "Gyro Calib    PRESS >"  = 21   ← 原来这里是 27，光标会被截掉 */

        case FMT_HZ:
            sprintf(line, "%-11s%4d.%dHz%s", cfg->name,
                    (int)(*cfg->val / 10), (int)(*cfg->val % 10), end);
            break;

        case FMT_ONOFF:
            sprintf(line, "%-11s%4s%s", cfg->name, (*cfg->val != 0) ? "ON" : "OFF", end);
            break;

        case FMT_AXIS:
            sprintf(line, "%-11s%4s%s", cfg->name,
                    AXIS_NAME[(*cfg->val >= 0 && *cfg->val < AXIS_NUM) ? *cfg->val : 0], end);
            break;

        /* ---- 姿态角：1 位小数的定点整数显示，避开 %f ----
         *   "Roll          12.34 >"  = 22 */
        case FMT_ROLL:
        case FMT_PITCH:
        case FMT_YAW:
        {
            char ang[12];
            int32_t v;

            if (cfg->fmt == FMT_ROLL)       { v = (int32_t)(att_roll  * 100.0f); }
            else if (cfg->fmt == FMT_PITCH) { v = (int32_t)(att_pitch * 100.0f); }
            else                            { v = (int32_t)(att_yaw   * 100.0f); }

            Menu2_Fixed1(ang, v, 100);
            sprintf(line, "%-11s%8s deg%s", cfg->name, ang, end);
            break;
        }

        case FMT_ACTION:
        case FMT_STR:
        default:
            sprintf(line, "%-11s%4s%s", cfg->name,
                    (cfg->str != NULL) ? cfg->str : "", end);
            break;
    }

    /* 选中行"反白"：整行蓝底白字，一眼看出光标停在哪一项；其余行白底黑字。
     * 配色由 Menu2_Row() 自己按 highlight 设置并恢复，这里不用管。 */
    Menu2_Row(row, line, selected);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     页面标题（按 s_page 指向的节点下标判断是哪个页面）
*------------------------------------------------------------------------------------------------------------------*/
static const char *Menu2_TitleText(uint8_t is_root)
{
    if (is_root != 0U)
    {
        return "Main Menu";
    }

    if (s_page == &s_nodes[PAGE_ID_AXIS])
    {
        /* 六轴页：IMU 掉线时标题里直接标出来 —— 不然看到一整页 0
         * 分不清是"传感器没接上"还是"数据真的是 0"。
         * 在线时标题顺带说明两行数据的单位（加速度 mg / 角速度 dps）。 */
        return (NEUQ_IMU_Online() != 0U) ? "6-AXIS   mg / dps" : "6-AXIS  IMU:OFF";
    }
    if (s_page == &s_nodes[PAGE_ID_WAVE]) { return "WAVE / FILTER"; }
    if (s_page == &s_nodes[PAGE_ID_ATT])
    {
        /* 陀螺校准状态借标题显示（校准期间标题会变，一眼能看出还在采零偏） */
        return (NEUQ_IMU_CalibBusy() != 0U) ? "ATTITUDE CALIBRATING"
                                            : "ATTITUDE   deg    C:done";
    }
    return "?";
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     整页重画
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Menu2_Redraw(void)
{
    Menu2Node *p;
    uint8_t    row   = 1U;              /* 第 0 行是标题；数据行从第 1 行（y = 24）开始 */
    uint8_t    is_root;

    /* 整屏 Clear() 只在"换页 / 首屏 / 显示过临时信息页"时做一次；
     * 平时靠行缓存，只重画变化的字符。 */
    if (s_need_clear != 0U)
    {
        NEUQ_LCD200_Clear();
        Menu2_CellsInvalidate();
        s_need_clear = 0U;
    }

    /* 一级功能选择页：s_page 是哨兵 [0]，它的 child 就是三个入口。
     * 二级页：s_page 是某个入口节点，它的 child 才是本页的项。 */
    is_root = (s_page == &s_nodes[PAGE_STR]) ? 1U : 0U;

    Menu2_Row(0U, Menu2_TitleText(is_root), 0U);

    p = s_page->child;
    if (p != NULL)
    {
        Menu2Node *first = p;

        do
        {
            Menu2_DrawItem(row, p->cfg);
            row++;
            p = p->next;
        } while (p != first);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     临时信息页（启动提示、保存结果等）
* 备注信息     会整屏清一次，所以行缓存要作废，并标一个"下次重画要再清一次"的标记。
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Menu2_ShowMessage(const char *l1, const char *l2)
{
    NEUQ_LCD200_Clear();
    Menu2_CellsInvalidate();
    s_need_clear = 1U;                          /* 临时页把整屏弄花了：下次重画强制再清一次 */
    NEUQ_LCD200_SetPenColor(LCD_BLACK);         /* 两行都黑字：原来第二行是白色，白底白字看不见 */
    if (l1 != NULL) { NEUQ_LCD200_DisplayString(0, 100, (char *)l1); }
    if (l2 != NULL) { NEUQ_LCD200_DisplayString(0, 130, (char *)l2); }
}

/*===================================================================================================================
*                                          四、按键处理
*==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     动作项：存 Flash / 恢复默认 / 陀螺零偏校准
*------------------------------------------------------------------------------------------------------------------*/
static void Menu2_RunAction(uint8_t act)
{
    switch (act)
    {
        case ACT_SAVE:
            NEUQ_Menu2_ShowMessage("SAVED TO FLASH",
                                   Menu2_Save() ? "sector5 @0x08020000" : "SAVE FAILED");
            NEUQ_Delay_ms(700);
            break;

        case ACT_DEFAULT:
            Menu2_Default();
            NEUQ_Menu2_ShowMessage("DEFAULT LOADED", "press Save to keep");
            NEUQ_Delay_ms(700);
            break;

        case ACT_CALIB:
            NEUQ_IMU_Calib();
            NEUQ_Menu2_ShowMessage("GYRO CALIBRATING", "keep module STILL");
            NEUQ_Delay_ms(1800);            /* 1.5 s 校准完再回来 */
            break;

        default:
            break;
    }
    NEUQ_Menu2_Redraw();
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     改一档参数
*------------------------------------------------------------------------------------------------------------------*/
static void Menu2_Adjust(const Menu2ItemConfig *cfg, int32_t dir)
{
    int32_t v = *cfg->val + dir * cfg->step;

    if (v < cfg->min) { v = cfg->min; }
    if (v > cfg->max) { v = cfg->max; }
    *cfg->val = v;

    if (cfg->fmt == FMT_HZ)
    {
        NEUQ_IMU_SetFc((uint16_t)v);        /* 截止频率立刻生效，波形马上能看到变化 */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     按键事件 → 菜单行为
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Menu2_HandleKey(KeyIdEnum id, KeyEventEnum event)
{
    const Menu2ItemConfig *cfg;

    if (event == KEY_EVENT_NONE || s_sel == NULL)
    {
        return;
    }

    if (id == KEY_ID_K0)
    {
        id = KEY_ID_FORWARD;                /* 板载 PA0 当确认键 */
    }

    cfg = s_sel->cfg;

    /* ---------------- 上 / 下 ---------------- */
    if (id == KEY_ID_UP || id == KEY_ID_DOWN)
    {
        int32_t dir = (id == KEY_ID_UP) ? 1 : -1;

        if (s_edit != 0)
        {
            Menu2_Adjust(cfg, dir);         /* 编辑中：改数值 */
        }
        else
        {
            Menu2_Move(dir);                /* 普通状态：移动光标（链表绕回） */
        }
        NEUQ_Menu2_Redraw();
        return;
    }

    /* ---------------- 进入 / 确认 / 执行 ---------------- */
    if (id == KEY_ID_FORWARD)
    {
        if (s_edit != 0)
        {
            s_edit = 0;                     /* 编辑确认：改动就留在 s_axis/s_send/s_cutoff 里 */
        }
        else if (cfg->fmt == FMT_ACTION)
        {
            Menu2_RunAction(cfg->act);
            return;
        }
        else if (s_sel->child != NULL)
        {
            s_page = s_sel;                 /* 进子页：光标先落在第一个子项上 */
            s_sel  = s_sel->child;
            s_need_clear = 1U;              /* 换页：整屏清一次（项数不同的页会留残行） */
        }
        else if (cfg->step != 0)
        {
            s_edit = 1;                     /* 可编辑项：进入编辑 */
            s_old  = *cfg->val;             /* 记下原值，BACK 时回滚 */
        }
        NEUQ_Menu2_Redraw();
        return;
    }

    /* ---------------- 返回 / 取消 ---------------- */
    if (id == KEY_ID_BACK)
    {
        if (s_edit != 0)
        {
            *cfg->val = s_old;              /* 编辑取消：数值回滚 */
            if (cfg->fmt == FMT_HZ)
            {
                NEUQ_IMU_SetFc((uint16_t)s_old);
            }
            s_edit = 0;
        }
        else if (s_page->parent != NULL)
        {
            /* 返回上级：退到上一级页锚点，光标就停在刚才进来的那一项上
             * （入口节点自己既是锚点又是菜单项，所以直接就是它）。 */
            s_sel  = s_page;
            s_page = s_page->parent;
            s_need_clear = 1U;              /* 换页：整屏清一次（项数不同的页会留残行） */
        }
        NEUQ_Menu2_Redraw();
        return;
    }
}

/*===================================================================================================================
*                                          五、VOFA+ 上传
*==================================================================================================================*/

/* 浮点 -> 两位小数文本（不用 %f；负数按数值方向四舍五入）。 */
static uint8_t Menu2_FwFixed2(char *dst, float v)
{
    int32_t x;

    if (v > 3000000.0f)  { v =  3000000.0f; }
    if (v < -3000000.0f) { v = -3000000.0f; }

    x = (int32_t)((v >= 0.0f) ? (v * 100.0f + 0.5f)
                              : (v * 100.0f - 0.5f));
    if (x < 0)
    {
        x = -x;
        return (uint8_t)sprintf(dst, "-%ld.%02ld",
                                (long)(x / 100), (long)(x % 100));
    }
    return (uint8_t)sprintf(dst, "%ld.%02ld", (long)(x / 100), (long)(x % 100));
}

/* 发一帧 FireWater 文本："原始值,滤波值\n"，只发当前选中的一轴。 */
static void Menu2_VofaSend(void)
{
    static uint32_t last = 0;
    char    line[40];
    float   raw_v;
    float   filt_v;
    uint8_t used;

    if (s_send == 0 || (uint32_t)(HAL_GetTick() - last) < 10U)
    {
        return;                             /* 关掉上传只是不发，采集和滤波照常 */
    }
    last = HAL_GetTick();

    /* ★ 原始值与滤波值必须来自**同一次采样**。
     *   这两个数组由 TIM7 的 5 ms 中断更新，中断里先写 imu_raw[]、再写 imu_filt[]；
     *   主循环如果读在两次写之间，就会把"新原始值 + 旧滤波值"配成一帧。
     *   关中断只包住两次 float 读取（几条指令），不会影响 5 ms 节拍。 */
    __disable_irq();
    raw_v  = imu_raw[s_axis];
    filt_v = imu_filt[s_axis];
    __enable_irq();

    used  = Menu2_FwFixed2(line, raw_v);                    /* ch0：原始计数 */
    line[used] = ',';                                       /* 通道分隔符 */
    used++;
    used += Menu2_FwFixed2(&line[used], filt_v);            /* ch1：同一次采样的滤波值 */
    line[used] = '\n';                                      /* 换行 = 帧尾 */
    used++;
    line[used] = '\0';

    NEUQ_Uart_write_data(VOFA_UART, (uint8_t *)line, (uint32_t)used);
}

/*===================================================================================================================
*                                          六、初始化与任务
*==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     主循环任务：每 100 ms 刷一遍当前页 + 按需上传
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Menu2_Task(void)
{
    if ((uint32_t)(HAL_GetTick() - s_draw) >= 100U)
    {
        s_draw = HAL_GetTick();
        NEUQ_Menu2_Redraw();
    }
    Menu2_VofaSend();
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     初始化：LCD + 建链 + 读 Flash 参数 + 首屏
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Menu2_Init(void)
{
    static Menu2Node nodes[NODE_NUM];

    s_nodes = nodes;

    NEUQ_LCD200_init();

    Menu2_Build();
    Menu2_Load();                       /* 读 Flash：无有效记录就用默认值 */

    NEUQ_Uart_init(VOFA_UART, UART_BAUD_115200);   /* VOFA+ 串口（默认 USART1/PA9） */

    s_page = &s_nodes[PAGE_STR];        /* 一级功能选择页 = 哨兵 */
    s_sel  = s_nodes[PAGE_STR].child;   /* 光标落在第一个入口（哨兵 child = 6-Axis） */
    s_edit = 0;
    s_draw = HAL_GetTick();

    s_need_clear = 1U;                  /* 首屏：整屏清一次，并把行缓存作废 */
    NEUQ_Menu2_Redraw();
}
