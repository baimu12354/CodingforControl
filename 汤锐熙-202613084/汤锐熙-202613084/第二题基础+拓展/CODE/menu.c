/*********************************************************************************************************************
* 文件名称   menu
* 功能说明   编码式菜单实现：页面与参数都“编码”进结构体，引擎统一解释。
*            加参数：在 config.h 结构体加字段 + WaveItems 表加一行，引擎不用改。
*********************************************************************************************************************/
#include "menu.h"
#include "config.h"

/* ============ 菜单项编码 ============ */
typedef struct MenuPage MenuPage;

typedef enum { ITEM_PAGE, ITEM_INT } MenuItemType_t;   // 目录项 / 可编辑整数

typedef struct {
    char            *name;      // 显示文字
    uint8_t          type;      // ITEM_PAGE 或 ITEM_INT
    void            *value;     // ITEM_INT：指向待编辑变量
    int32_t          min, max, step;
    char            *unit;      // 单位后缀，可为空
    const MenuPage  *child;     // ITEM_PAGE：指向子页
} MenuItem_t;

struct MenuPage {
    char             *title;
    const MenuItem_t *items;
    uint8_t           count;    // count==0 表示纯显示页(六轴/姿态)
    void (*render)(const MenuPage *self, uint8_t cursor);   // 显示页自定义画法
};

/* ============ 页面定义 ============ */
static void Page_SixAxis_Render(const MenuPage *p, uint8_t c);
static void Page_Attitude_Render(const MenuPage *p, uint8_t c);

static const MenuPage Page_SixAxis  = { "SixAxis",   NULL, 0, Page_SixAxis_Render };
static const MenuPage Page_Attitude = { "Attitude",  NULL, 0, Page_Attitude_Render };

// 波形与滤波设置页：可编辑参数直接绑定 g_cfg 字段
static const MenuItem_t WaveItems[] = {
    { "SendEnable", ITEM_INT, &g_cfg.send_enable,  0,    1,  1, "",   NULL },
    { "SelectAxis", ITEM_INT, &g_cfg.selected_axis,0,    5,  1, "",   NULL },
    { "LPF alpha",  ITEM_INT, &g_cfg.alpha_x1000,  0, 1000, 10, "",   NULL },
};
static const MenuPage Page_Wave = { "Wave&Filter", WaveItems, 3, NULL };

// 一级主菜单
static const MenuItem_t RootItems[] = {
    { "SixAxis",     ITEM_PAGE, NULL, 0, 0, 0, NULL, &Page_SixAxis },
    { "Wave&Filter", ITEM_PAGE, NULL, 0, 0, 0, NULL, &Page_Wave },
    { "Attitude",    ITEM_PAGE, NULL, 0, 0, 0, NULL, &Page_Attitude },
};
static const MenuPage Page_Root = { "Main Menu", RootItems, 3, NULL };

/* ============ 引擎内部状态 ============ */
static const MenuPage *g_cur      = &Page_Root;
static uint8_t         g_cursor   = 0;
static int             g_edit     = -1;    // -1=浏览；>=0=正在编辑 items[g_edit]
static int32_t         g_edit_tmp = 0;
static const MenuPage *g_parent   = NULL;
static uint8_t         g_full_redraw = 1;   // 1=需要整屏重画(进入页面/按键后)，显示页据此决定是否清屏

/* ============ 对外函数 ============ */
void Menu_Init(void)
{
    g_cur    = &Page_Root;
    g_cursor = 0;
    g_edit   = -1;
    g_parent = NULL;
    NEUQ_LCD200_SetBackColor(LCD_BLACK);
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    NEUQ_LCD200_Clear();
    Menu_Render();
}

void Menu_OnEvent(KeyEvent_t ev)
{
    if (ev == KEY_NONE) return;
    g_full_redraw = 1;          // 有按键，画面要变，标记需要整屏重画

    // 纯显示页(六轴/姿态)：只响应“返回”
    if (g_cur->count == 0) {
        if (ev == KEY_BACK && g_parent) { g_cur = g_parent; g_parent = NULL; }
        return;
    }

    // 编辑模式
    if (g_edit >= 0) {
        const MenuItem_t *it = &g_cur->items[g_edit];
        if (ev == KEY_UP)   g_edit_tmp += it->step;
        if (ev == KEY_DOWN) g_edit_tmp -= it->step;
        if (g_edit_tmp > it->max) g_edit_tmp = it->max;   // 夹紧到上下限
        if (g_edit_tmp < it->min) g_edit_tmp = it->min;

        if (ev == KEY_OK) {                 // 确认：写回变量 + 触发保存
            *(int32_t *)it->value = g_edit_tmp;
            g_save_flag = 1;
            g_edit = -1;
        } else if (ev == KEY_BACK) {        // 取消：丢弃本次修改
            g_edit = -1;
        }
        return;
    }

    // 浏览模式
    const MenuItem_t *it = &g_cur->items[g_cursor];
    if (ev == KEY_UP)   { g_cursor = (uint8_t)((g_cursor + g_cur->count - 1) % g_cur->count); }
    if (ev == KEY_DOWN) { g_cursor = (uint8_t)((g_cursor + 1) % g_cur->count); }
    if (ev == KEY_OK) {
        if (it->type == ITEM_PAGE) {        // 进入子页
            g_parent = g_cur;
            g_cur    = it->child;
            g_cursor = 0;
        } else if (it->type == ITEM_INT) {  // 进入编辑
            g_edit     = (int)g_cursor;
            g_edit_tmp = *(int32_t *)it->value;
        }
    }
    if (ev == KEY_BACK && g_parent) {       // 返回上一级
        g_cur    = g_parent;
        g_parent = NULL;
        g_cursor = 0;
    }
}

void Menu_Render(void)
{
    if (g_cur->render) { g_cur->render(g_cur, g_cursor); return; }   // 显示页

    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(0, 0, g_cur->title);

    for (uint8_t i = 0; i < g_cur->count; i++) {
        const MenuItem_t *it = &g_cur->items[i];
        char line[32];
        if (it->type == ITEM_INT) {
            int32_t v = (g_edit == (int)i) ? g_edit_tmp : *(int32_t *)it->value;
            sprintf(line, "%c%s %d%s", (i == g_cursor) ? '>' : ' ',
                    it->name, v, it->unit ? it->unit : "");
        } else {
            sprintf(line, "%c%s", (i == g_cursor) ? '>' : ' ', it->name);
        }
        NEUQ_LCD200_DisplayString(0, 24 + i * 24, line);   // 标题高24px(占0~23)，条目从24起，避免重叠
    }
}

void Menu_RefreshDisplay(void)
{
    static uint32_t last = 0;
    if (g_cur->render == NULL) return;              // 列表页不周期刷
    if ((HAL_GetTick() - last) >= 500) {             // 显示页每50ms刷
        last = HAL_GetTick();
        Menu_Render();
    }
}

/* ============ 显示页自定义画法 ============ */

// 浮点 -> 定宽字符串：保留 dec 位小数，右对齐到 width 字符（不足补空格）。
// 本工程 MicroLIB 不支持 %f，所以用整数+小数手工拼，避免用到 printf 的浮点格式。
// 注意 width 要 >= 数值最长的字符数（加速度2位小数最长5字符、角速度3位小数最长7字符）。
static void FmtFloat(char *buf, float v, int dec, int width)
{
    char tmp[16], frac[8];
    int  scale = 1, t, ip, fp, len, i;

    for (i = 0; i < dec; i++) scale *= 10;
    t = (int)(v * scale + (v < 0 ? -0.5f : 0.5f));   // 四舍五入成整数

    if (t < 0) { ip = -t / scale; fp = -t % scale; }
    else       { ip =  t / scale; fp =  t % scale; }

    for (i = dec - 1; i >= 0; i--) { frac[i] = (char)('0' + fp % 10); fp /= 10; }
    frac[dec] = '\0';

    sprintf(tmp, "%s%d.%s", (t < 0) ? "-" : "", ip, frac);

    len = strlen(tmp);
    for (i = 0; i < width - len; i++) buf[i] = ' ';   // 前面补空格 -> 右对齐
    strcpy(buf + (width - len), tmp);
    buf[width] = '\0';
}

static void Page_SixAxis_Render(const MenuPage *p, uint8_t c)
{
    (void)p; (void)c;
    char buf[16];

    if (g_full_redraw) {                 // 进入页面：清屏+画静态文字（标签、单位，只画一次）
        g_full_redraw = 0;
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_DisplayString(0, 0, "SixAxis");
        NEUQ_LCD200_DisplayString(0,  24, "ax :");  NEUQ_LCD200_DisplayString(156,  24, "g");
        NEUQ_LCD200_DisplayString(0,  48, "ay :");  NEUQ_LCD200_DisplayString(156,  48, "g");
        NEUQ_LCD200_DisplayString(0,  72, "az :");  NEUQ_LCD200_DisplayString(156,  72, "g");
        NEUQ_LCD200_DisplayString(0,  96, "gx :");  NEUQ_LCD200_DisplayString(156,  96, "rad/s");
        NEUQ_LCD200_DisplayString(0, 120, "gy :");  NEUQ_LCD200_DisplayString(156, 120, "rad/s");
        NEUQ_LCD200_DisplayString(0, 144, "gz :");  NEUQ_LCD200_DisplayString(156, 144, "rad/s");
        NEUQ_LCD200_DisplayString(0, 200, "[BACK] return");
    }
    // 每次只刷数值：固定x、固定宽度(8位右对齐)，盖写旧值不留残影
    FmtFloat(buf, g_acc_g[0], 2, 8);    NEUQ_LCD200_DisplayString(48,  24, buf);
    FmtFloat(buf, g_acc_g[1], 2, 8);    NEUQ_LCD200_DisplayString(48,  48, buf);
    FmtFloat(buf, g_acc_g[2], 2, 8);    NEUQ_LCD200_DisplayString(48,  72, buf);
    FmtFloat(buf, g_gyro_rad[0], 3, 8); NEUQ_LCD200_DisplayString(48,  96, buf);
    FmtFloat(buf, g_gyro_rad[1], 3, 8); NEUQ_LCD200_DisplayString(48, 120, buf);
    FmtFloat(buf, g_gyro_rad[2], 3, 8); NEUQ_LCD200_DisplayString(48, 144, buf);
}

// 把浮点角格式化成“一位小数”字符串（本工程用 MicroLIB，不支持 %f，故手工拼）
static void FmtAngle1(char *buf, float v)
{
    int t = (int)(v * 10.0f + (v < 0 ? -0.5f : 0.5f));   // 四舍五入到 0.1 度
    int neg = (t < 0);
    if (neg) t = -t;
    sprintf(buf, "%s%d.%d", neg ? "-" : "", t / 10, t % 10);
}

static void Page_Attitude_Render(const MenuPage *p, uint8_t c)
{
    (void)p; (void)c;
    char num[16], buf[24];

    if (g_full_redraw) {                 // 进入页面：清屏+画静态文字（只画一次）
        g_full_redraw = 0;
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_DisplayString(0, 0, "Attitude");
        NEUQ_LCD200_DisplayString(0,  24, "roll :");
        NEUQ_LCD200_DisplayString(0,  48, "pitch:");
        NEUQ_LCD200_DisplayString(0,  72, "yaw  :");
        NEUQ_LCD200_DisplayString(0, 200, "[BACK] return");
    }
    // 每次只刷数值：固定x、固定宽度(6位左对齐)，盖写旧值不留残影
    FmtAngle1(num, g_roll);   sprintf(buf, "%-6s", num);  NEUQ_LCD200_DisplayString(72, 24, buf);
    FmtAngle1(num, g_pitch);  sprintf(buf, "%-6s", num);  NEUQ_LCD200_DisplayString(72, 48, buf);
    FmtAngle1(num, g_yaw);    sprintf(buf, "%-6s", num);  NEUQ_LCD200_DisplayString(72, 72, buf);
}
