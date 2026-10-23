#include "menu.h"
#include "imu.h"
#include "param.h"
#include <stdarg.h>

static const char *axis_names[AX_NUM] = {"Ax","Ay","Az","Gx","Gy","Gz"};
static const char *axis_units[AX_NUM] = {"g","g","g","deg/s","deg/s","deg/s"};

/* 菜单节点：主菜单项构成双向循环链表 */
typedef struct Menu_Node {
    const char      *name;     /* 显示名称 */
    Page_Id          page;     /* 进入的目标页面 */
    struct Menu_Node *prev;    /* 上一个菜单项 */
    struct Menu_Node *next;    /* 下一个菜单项 */
} Menu_Node;

/* 主菜单三项（双向循环链表），menu_current 指向当前高亮项 */
static Menu_Node menu_data   = {"1. Data Display", PAGE_DATA,    NULL, NULL};
static Menu_Node menu_filter = {"2. Filter Setup", PAGE_SETTING, NULL, NULL};
static Menu_Node menu_att    = {"3. Attitude",     PAGE_ATTITUDE, NULL, NULL};
static Menu_Node *menu_current = &menu_data;

Page_Id page = PAGE_MAIN;
static uint8_t set_cursor, edit_mode;
static const GPIO_Ports_ENUM key_pins[KEY_NUM] = {KEY_UP_PIN,KEY_DOWN_PIN,KEY_ENTER_PIN,KEY_BACK_PIN};

static uint8_t key_filt[KEY_NUM], key_state[KEY_NUM];
static uint32_t key_press_tick[KEY_NUM];

static void LCD_Printf(uint16_t x, uint16_t y, const char *fmt, ...) {
    char buf[48]; va_list ap;
    va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    NEUQ_LCD200_DisplayString(x, y, buf);
}

/* 按键：软件去抖 + 长按重复，返回 0=无 1=按下 2=长按重复 */
uint8_t Key_Read(Key_Id id) {
    uint8_t p = (NEUQ_GPIO_read(key_pins[id]) == KEY_ACTIVE_LEVEL), e = 0;
    key_filt[id] = p ? (key_filt[id] < 251 ? key_filt[id] + 4 : 255)
                     : (key_filt[id] > 4 ? key_filt[id] - 4 : 0);
    uint8_t s = key_filt[id] >= 0x80;
    if (s && !key_state[id]) { e = 1; key_press_tick[id] = time_tick; }
    else if (s && key_state[id] && time_tick - key_press_tick[id] >= 500) { e = 2; key_press_tick[id] += 150; }
    key_state[id] = s;
    return e;
}

/* ================= 页面绘制 ================= */
static void Title(const char *s) {
    NEUQ_LCD200_Clear(); NEUQ_LCD200_SetPenColor(LCD_BLACK); NEUQ_LCD200_DisplayString(60, 10, (char *)s);
}

/* 主菜单链表初始化：三项连成双向循环链表 */
void Menu_Init(void) {
    NEUQ_GPIO_init(KEY_UP_PIN,    GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_DOWN_PIN,  GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_ENTER_PIN, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_BACK_PIN,  GPIO_Mode_Input_PullUp, GPIO_High);

    menu_data.prev   = &menu_att;    menu_data.next   = &menu_filter;
    menu_filter.prev = &menu_data;   menu_filter.next = &menu_att;
    menu_att.prev    = &menu_filter; menu_att.next    = &menu_data;
}

static void Draw_Main(void) {
    Title("IMU MENU");
    Menu_Node *m = &menu_data; int y = 70;
    do {
        LCD_Printf(40, y, "%c %s", m == menu_current ? '*' : ' ', m->name);
        m = m->next; y += 40;
    } while (m != &menu_data);
}

static void Draw_Data(void) {
    Title("6-AXIS DATA");
    for (int i = 0; i < AX_NUM; i++)
    LCD_Printf(20, 45 + i * 30, "%s: %+8.3f %s", axis_names[i], raw_axis[i], axis_units[i]);
    NEUQ_LCD200_DisplayString(20, 225, "BACK: return");
}

static void Draw_Setting(void) {
    Title("FILTER SETUP");
    LCD_Printf(20, 45,  "%c Axis  : %s", set_cursor == 0 ? '*' : ' ', axis_names[param.send_axis]);
    LCD_Printf(20, 75,  "%c Upload: %s", set_cursor == 1 ? '*' : ' ', param.send_enable ? "ON" : "OFF");
    LCD_Printf(20, 105, "%c Filter: %3u%%", set_cursor == 2 ? '*' : ' ', param.filter_percent);
    LCD_Printf(20, 135, "%c Save to Flash", set_cursor == 3 ? '*' : ' ');
    if (time_tick - last_save_tick < 1500) NEUQ_LCD200_DisplayString(20, 165, "Saved OK");
    NEUQ_LCD200_DisplayString(20, 210, "ENTER:edit  BACK:return");
}

static void Draw_Attitude(void) {
    Title("ATTITUDE");
    NEUQ_LCD200_DisplayString(40, 70,  "Roll :");
    NEUQ_LCD200_DisplayString(40, 100, "Pitch:");
    NEUQ_LCD200_DisplayString(40, 130, "Yaw  :");
    NEUQ_LCD200_DisplayString(20, 205, "ENTER: zero yaw  BACK: return");
}

/* 动态刷新：只重绘数值区域 */
void Refresh_Values(void) {
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16); NEUQ_LCD200_SetPenColor(LCD_BLACK);
    if (page == PAGE_DATA) {
        for (int i = 0; i < AX_NUM; i++) { NEUQ_LCD200_ClearRect(20, 45 + i * 30, 280, 8);
             LCD_Printf(20, 45 + i * 30, "%s: %+8.3f %s", axis_names[i], raw_axis[i], axis_units[i]); }
    } else if (page == PAGE_SETTING) {
        NEUQ_LCD200_ClearRect(20, 185, 280, 8); LCD_Printf(20, 185, "Raw: %+8.3f  Flt: %+8.3f", raw_axis[param.send_axis], filt_axis[param.send_axis]);
    } else if (page == PAGE_ATTITUDE) {
        NEUQ_LCD200_ClearRect(120, 70, 120, 8);  LCD_Printf(120, 70,  "%+7.2f deg", att_roll);
        NEUQ_LCD200_ClearRect(120, 100, 120, 8); LCD_Printf(120, 100, "%+7.2f deg", att_pitch);
        NEUQ_LCD200_ClearRect(120, 130, 120, 8); LCD_Printf(120, 130, "%+7.2f deg", att_yaw);
    }
}

/* 进入页面：切换当前页并触发该页的首次绘制 */
void Page_Enter(Page_Id p) {
    page = p; edit_mode = 0;
    switch (page) {
        case PAGE_MAIN:     Draw_Main();     break;
        case PAGE_DATA:     Draw_Data();     break;
        case PAGE_SETTING:  Filter_Reset(); Draw_Setting(); break;
        case PAGE_ATTITUDE: Draw_Attitude(); break;
    }
}

void Handle_Keys(uint8_t up, uint8_t down, uint8_t enter, uint8_t back) {
    switch (page) {
        case PAGE_MAIN:
            if (up) { menu_current = menu_current->prev; Draw_Main(); }
            else if (down) { menu_current = menu_current->next; Draw_Main(); }
            if (enter) Page_Enter(menu_current->page);
            break;
        case PAGE_DATA:
            if (back) Page_Enter(PAGE_MAIN);
            break;
        case PAGE_SETTING:
            if (!edit_mode) {
                if (up) { set_cursor = (set_cursor + 3) % 4; Draw_Setting(); }
                else if (down) { set_cursor = (set_cursor + 1) % 4; Draw_Setting(); }
                if (enter) {
                    if (set_cursor == 3) { Param_Save(); Draw_Setting(); }
                    else if (set_cursor == 1) { param.send_enable = !param.send_enable; Draw_Setting(); }
                    else { edit_mode = 1; Draw_Setting(); }
                }
                if (back) Page_Enter(PAGE_MAIN);
            } else {
                if (up) {
                    if (set_cursor == 0) param.send_axis = (uint8_t)((param.send_axis + 5) % AX_NUM);
                    else if (set_cursor == 2) { if (param.filter_percent < 100) param.filter_percent++; }
                    Draw_Setting();
                }
                else if (down) {
                    if (set_cursor == 0) param.send_axis = (uint8_t)((param.send_axis + 1) % AX_NUM);
                    else if (set_cursor == 2) { if (param.filter_percent > 1) param.filter_percent--; }
                    Draw_Setting();
                }
                if (enter || back) { edit_mode = 0; Draw_Setting(); }
            }
            break;
        case PAGE_ATTITUDE:
            if (enter) Zero_Yaw();
            if (back) Page_Enter(PAGE_MAIN);
            break;
    }
}
