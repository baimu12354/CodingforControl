#include "NEUQ_lib_include.h"

NEUQ_MENU_PARAMS  params;
static   uint8_t  show_save_tip = 0;                 //1：正在显示保存提示 0：不显示
static   uint32_t save_tip_start_tick = 0;           //记录保存提示时间点
static   uint8_t  full_redraw = 1;                   //进入新页面需要整屏清除一次再重绘
volatile uint32_t sys_tick_10ms;                     //10ms 系统节拍
volatile uint8_t  lcd_refresh_flag;                  //LCD 定时刷新标志

typedef enum {
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_DATA,          // 一级：六轴数据显示
    MENU_WAVE,          // 一级：波形与滤波设置
    MENU_WAVE_AXIS,     // 二级：选择轴
    MENU_WAVE_SEND,     // 二级：发送开关
    MENU_WAVE_ALPHA,    // 二级：滤波参数
    MENU_WAVE_SAVE,     // 二级：保存到Flash
    MENU_ATTITUDE,      // 一级：姿态解算
    MENU_COUNT,
} MENUID;

//页面信息
typedef struct {
    const char *name;
    MENUID    parent_id;
    int32_t   *value;//当前页面参数地址
    int32_t   min;  //参数最小值
    int32_t   max;  //参数最大值
    int32_t   step; //每次增量
}MENUCONFIG;

static const MENUCONFIG MENUTABLE[MENU_COUNT] = {//节点内容
    // 一级菜单
    [MENU_MAIN]       = {"Main Menu",       MENU_NONE,NULL,               0, 0, 0},
    [MENU_DATA]       = {"6-Axis Data",     MENU_MAIN,NULL,               0, 0, 0}, // 无参数，纯显示
    [MENU_WAVE]       = {"Wave & Filter",   MENU_MAIN,NULL,               0, 0, 0}, // 波形与滤波设置
    [MENU_ATTITUDE]   = {"Attitude Solve",  MENU_MAIN,NULL,               0, 0, 0}, // 无参数，纯显示
    //二级菜单
    [MENU_WAVE_AXIS]  = {"Select Axis",     MENU_WAVE,&params.wave_axis,   0, 5,   1},//选择轴
    [MENU_WAVE_SEND]  = {"Send Enable",     MENU_WAVE,&params.send_enable, 0, 1,   1},//控制VOFA+发送开关
    [MENU_WAVE_ALPHA] = {"Filter Alpha",    MENU_WAVE,&params.filter_alpha,0, 100, 1},//控制滤波系数
    [MENU_WAVE_SAVE]  = {"Save to Flash",   MENU_WAVE,NULL,                0, 0,   0},
};

typedef struct MENUNODE{
    const MENUCONFIG *info;  //节点内容
    struct MENUNODE *parent; //层级父节点
    struct MENUNODE *child;  //层级子节点
    struct MENUNODE *prev;   //同级前节点
    struct MENUNODE *next;
}MENUNODE;

static MENUNODE nodes[MENU_COUNT]; //固定内存存储节点
static MENUNODE *page;             //当前页面
static MENUNODE *selected;         //当前节点（selected永远是page的子节点）
static uint8_t  editing;           //0：浏览模式   1：编辑模式


static void MENU_Buildlinks(void){
    for (uint8_t i=0; i<MENU_COUNT;i++){
        nodes[i].info   = &MENUTABLE[i];
        nodes[i].parent = NULL;
        nodes[i].child  = NULL;
        nodes[i].prev   = NULL;
        nodes[i].next   = NULL;
    }
    for (uint8_t i=0; i<MENU_COUNT;i++){
        MENUNODE *node = &nodes[i];
        MENUNODE *parent;
        if(node->info->parent_id == MENU_NONE){
            continue;
        }
        parent = &nodes[node->info->parent_id];
        node->parent = parent;
        MENUNODE *first;
        MENUNODE *last;
        first = parent->child;
        if (first == NULL){
            parent->child = node;
            node->prev = node;
            node->next = node;
        }else {
            last = first->prev;
            node->prev = last;
            last->next = node;
            node->next = first;
            first->prev = node;
        }
    }
}


static int32_t edit_value = 0;//编辑状态的值

static uint8_t MENU_UPDATE(KEY_EVENT_ENUM key) {//返回1刷新屏幕
    if (selected == NULL && editing == 0){
        if(key == KEY_BACK_SHORT){
            if(page -> parent != NULL){
                page  = page -> parent;
                selected = page -> child;
                return 1;
            }
        }
        return 0;
    }
    if (editing == 1) {
        const MENUCONFIG *item = selected -> info;
        switch (key) {
            case KEY_UP_SHORT:
            case KEY_UP_LONG:
                edit_value += item -> step;
                break;
            case KEY_DOWN_SHORT:
            case KEY_DOWN_LONG:
                edit_value -= item -> step;
                break;
            case KEY_FORWARD_SHORT:              // 确认修改
                if (item->value != NULL) {
                    *(item->value) = edit_value;
                }
                editing = 0;
                break;
            case KEY_BACK_SHORT:        // 取消修改
                editing = 0;
                break;
            default:
                return 0;
        }
        if (edit_value < item->min) edit_value = item->min;
        if (edit_value > item->max) edit_value = item->max;
        return 1;
    }

    //浏览模式
    switch (key) {
        case KEY_UP_SHORT:
        case KEY_UP_LONG:
            selected = selected->prev;
            break;
        case KEY_DOWN_SHORT:
        case KEY_DOWN_LONG:
            selected = selected->next;
            break;
        case KEY_FORWARD_SHORT:     // 确认键
            if (selected->child != NULL) {
                page = selected;
                selected = selected->child;
            } 
            else if (selected->info->value != NULL) {
                // 有 value 指针，进入编辑模式
                editing = 1;
                edit_value = *(selected->info->value);
            }
            else if (selected == &nodes[MENU_WAVE_SAVE]) {
                uint8_t save_result = NEUQ_Flash_save(0x1FF000, (uint8_t *)&params, sizeof(params));
                if (save_result == NEUQ_FLASH_OK) {
                    show_save_tip = 1;              //保存成功后置1
                    save_tip_start_tick = sys_tick_10ms;
                }
            }
            else if (selected == &nodes[MENU_DATA] || selected == &nodes[MENU_ATTITUDE]){
                page = selected;
                selected = NULL;
            }
            break;
        case KEY_BACK_SHORT:                // 返回键
            if (page -> parent != NULL) {
                selected = page;
                page = page->parent;
            }
            break;
        default:
            return 0; // 没有有效按键，不刷新
    }
    return 1; // 有按键动作，需要刷新屏幕
}


// 显示逻辑
static uint8_t MENU_SHOW(void) {
    char line[48];
    //  保存提示页
    if (show_save_tip == 1) {
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_DisplayString(40, 80, "Saved to Flash!");
        return 1;
    }
    //  六轴数据显示页
    if (page == &nodes[MENU_DATA]) {
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        if (full_redraw == 1) { 
            full_redraw = 0;
            NEUQ_LCD200_Clear();
            NEUQ_LCD200_DisplayString(8,   8, "AccX:");
            NEUQ_LCD200_DisplayString(8,  40, "AccY:");
            NEUQ_LCD200_DisplayString(8,  72, "AccZ:");
            NEUQ_LCD200_DisplayString(8, 104, "GyroX:");
            NEUQ_LCD200_DisplayString(8, 136, "GyroY:");
            NEUQ_LCD200_DisplayString(8, 168, "GyroZ:");
        }
        // 加速度
        sprintf(line, "%.2fg", filtered_acc_x);
        NEUQ_LCD200_ClearRect(68,   8, 72, 24);
        NEUQ_LCD200_DisplayString(68,   8, line);
        sprintf(line, "%.2fg", filtered_acc_y);
        NEUQ_LCD200_ClearRect(68,  40, 72, 24);
        NEUQ_LCD200_DisplayString(68,  40, line);
        sprintf(line, "%.2fg", filtered_acc_z);
        NEUQ_LCD200_ClearRect(68,  72, 72, 24);
        NEUQ_LCD200_DisplayString(68,  72, line);
        // 角速度
        sprintf(line, "%.1fdps", filtered_gyro_x);
        NEUQ_LCD200_ClearRect(80, 104, 120, 24);
        NEUQ_LCD200_DisplayString(80, 104, line);
        sprintf(line, "%.1fdps", filtered_gyro_y);
        NEUQ_LCD200_ClearRect(80, 136, 120, 24);
        NEUQ_LCD200_DisplayString(80, 136, line);
        sprintf(line, "%.1fdps", filtered_gyro_z);
        NEUQ_LCD200_ClearRect(80, 168, 120, 24);
        NEUQ_LCD200_DisplayString(80, 168, line);
        return 1;
    }
    //  姿态解算页
    if (page == &nodes[MENU_ATTITUDE]) {
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        if (full_redraw == 1) {
            full_redraw = 0;
            NEUQ_LCD200_Clear();
            NEUQ_LCD200_DisplayString(8, 16, "Pitch:");
            NEUQ_LCD200_DisplayString(8, 48, "Roll :");
            NEUQ_LCD200_DisplayString(8, 80, "Yaw  :");
        }
        sprintf(line, "%.1f deg", attitude_pitch);
        NEUQ_LCD200_ClearRect(80, 16, 120, 24);
        NEUQ_LCD200_DisplayString(80, 16, line);
        sprintf(line, "%.1f deg", attitude_roll);
        NEUQ_LCD200_ClearRect(80, 48, 120, 24);
        NEUQ_LCD200_DisplayString(80, 48, line);
        sprintf(line, "%.1f deg", attitude_yaw);
        NEUQ_LCD200_ClearRect(80, 80, 120, 24);
        NEUQ_LCD200_DisplayString(80, 80, line);
        return 1;
    }

    //  普通菜单列表页
    MENUNODE *item = page->child;
    uint16_t y = 56;
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_Clear();
    sprintf(line, "Page: %s", page->info->name);    //！！！
    NEUQ_LCD200_DisplayString(8, 8, line);
    if (item != NULL) {
        do {
            const MENUCONFIG *info = item->info;
            if (item == selected) {
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE); 
                NEUQ_LCD200_ClearRect(8, y, 224, 24);
                if (editing == 1 && info->value != NULL) sprintf(line, "* %s:%d", info->name, (int)edit_value);
                else if (info->value != NULL) sprintf(line, "> %s:%d", info->name, (int)*(info->value));
                else sprintf(line, "> %s", info->name);
                NEUQ_LCD200_DisplayString(8, y, line);
            } else {
                NEUQ_LCD200_SetPenColor(LCD_BLACK);
                NEUQ_LCD200_SetBackColor(LCD_WHITE);
                NEUQ_LCD200_ClearRect(8, y, 224, 24);
                if (info->value != NULL) sprintf(line, "  %s:%d", info->name, (int)*(info->value));
                else sprintf(line, "  %s", info->name);
                NEUQ_LCD200_DisplayString(8, y, line);
            }
            y += 32;
            item = item->next;
        } while (item != page->child);
    }
    return 1;
}

void NEUQ_MENU_INIT(void) {
    uint8_t read_status = NEUQ_Flash_read(0x1FF000, (uint8_t *)&params, sizeof(params));
    if (read_status != NEUQ_FLASH_OK || params.filter_alpha < 0 || params.filter_alpha > 100 ||params.wave_axis < 0 || params.wave_axis > 5 ||params.send_enable < 0 || params.send_enable > 1) {
        params.wave_axis    = 0;     // 默认X轴加速度
        params.send_enable  = 0;     // 默认关闭发送
        params.filter_alpha = 50;    // 默认滤波系数 0.50
        NEUQ_Flash_save(0x1FF000, (uint8_t *)&params, sizeof(params));
    }
    MENU_Buildlinks();              //先连接
    page = &nodes[MENU_MAIN];
    selected = page->child;
    editing = 0;
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    MENU_SHOW();
}

void NEUQ_MENU_TASK(void) {
    if (show_save_tip == 1) {
        if ((sys_tick_10ms - save_tip_start_tick) >= 50) { //显示0.5S
            show_save_tip = 0;
            full_redraw = 1;
            MENU_SHOW();
        }
    }

    KEY_EVENT_ENUM key = NEUQ_KEY_EVENTGET();
    if (MENU_UPDATE(key) == 1) {
        full_redraw = 1;         // 有按键动作，需要整屏刷新
        MENU_SHOW();             // 有按键动作立刻刷新屏幕
    }

    // VOFA+ 波形发送
    static uint32_t last_vofa_tick = 0;
    if (sys_tick_10ms - last_vofa_tick >= 1) {
        last_vofa_tick = sys_tick_10ms;
        VOFA_Send_Task();
    }

    // 屏幕定时刷新(100ms)
    if (lcd_refresh_flag == 1) {
        lcd_refresh_flag = 0;    // 清除100ms中断标志位
        if (page == &nodes[MENU_DATA] || page == &nodes[MENU_ATTITUDE]) {
            MENU_SHOW();
        }
    }
}