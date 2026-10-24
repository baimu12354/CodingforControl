#include "NEUQ_lib_include.h"
#include "NEUQ_MENU.h"

/* 参数存储：直接使用 NEUQ 库的 W25Q16 外部 Flash */
#define PARAM_SECTOR  511     /* W25Q16 最后一个扇区，专存参数（4KB，足够） */

/* 读出的参数是否合法：全0xFF(-1)/越界/损坏 → 返回0 */
static uint8_t Params_IsValid(void){
    if (Params.send_axis   < 0 || Params.send_axis   > 5) return 0;
    if (Params.send_enable < 0 || Params.send_enable > 1) return 0;
    if (Params.alpha       < 5 || Params.alpha       > 95) return 0;
    return 1;
}


volatile NEUQ_Menu_Params Params;

//页面编号
typedef enum
{
    MENU_NONE = -1,
    MENU_MAIN = 0,   /* 一级菜单 */
    MENU_DATA,       /* 二级：六轴数据显示 */
    MENU_WAVE,       /* 二级：波形与滤波设置 */
    MENU_ATT,        /* 二级：姿态解算 */
    MENU_AXIS,       /* 三级：选择发送轴 */    
    MENU_SEND,       /* 三级：开启/停止波形发送 */    
    MENU_ALPHA,      /* 三级：修改滤波参数 */    
    MENU_COUNT
} MenuID;

//页面结构
typedef struct
{
    /* data */
    const char *name;   //当前页面名称
    MenuID parent_id;   //当前页面的父节点编号
    volatile int32_t *value;    //当前页面的参数地址(NULL表示该节点不编辑参数)
    int32_t min;
    int32_t max;
    int32_t step;
}MenuConfig;

static const MenuConfig MenuTable[MENU_COUNT] = {    
[MENU_MAIN]   = {"Main menu",     MENU_NONE, NULL,  0,  0, 0},    
[MENU_DATA]   = {"6-Axis Data",   MENU_MAIN, NULL,  0,  0, 0},    
[MENU_WAVE]   = {"Wave & Filter", MENU_MAIN, NULL,  0,  0, 0},    
[MENU_ATT]    = {"Attitude",      MENU_MAIN, NULL,  0,  0, 0},    
[MENU_AXIS]   = {"Send Axis",     MENU_WAVE, &Params.send_axis,     0,  5, 1},   
[MENU_SEND]   = {"Send",          MENU_WAVE, &Params.send_enable,   0,  1, 1},    
[MENU_ALPHA]  = {"Alpha",         MENU_WAVE, &Params.alpha,         5, 95, 1},
};

//链表节点
typedef struct MenuNode{
    const MenuConfig *info;     //节点内容
    struct MenuNode *parent;    //层级父节点
    struct MenuNode *child;     //层级子节点
    struct MenuNode *prev;      //同级前节点
    struct MenuNode *next;      //同级后节点
} MenuNode;

static MenuNode nodes[MENU_COUNT];   // 固定内存存储节点
static MenuNode *page;               // 当前页面 
static MenuNode *selected;           // 当前选项(selected 永远是 page 的子节点) 
static uint8_t  editing;             // 0:浏览模式 1:编辑模式 

//根据菜单表格去构建节点链接关系
static void Menu_BuildLinks(void){

    //第一步，赋值所有节点的页面信息
    for (uint8_t i=0; i<MENU_COUNT; i++){
        nodes[i].info = &MenuTable[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = NULL;
        nodes[i].next = NULL;
    }

    //第二步，构建各个节点之间的层级关系 和 同级关系
    for (uint8_t i=0; i<MENU_COUNT; i++){
        //构建层级关系
        MenuNode *node = &nodes[i];
        MenuNode *parent;
        if (node->info->parent_id == MENU_NONE) continue;
        parent = &nodes[node->info->parent_id];
        node->parent = parent;

        //构建同级关系
        MenuNode *first;
        MenuNode *last;
        first =  parent->child;
        if (first == NULL){
            parent->child = node;
            node->prev = node;
            node->next = node;
        } else{
            last = first->prev;
            node->prev = last;
            last->next = node;
            node->next = first;
            first->prev = node;
        }
    }
}

int32_t edit_value = 0;
static int32_t edit_backup = 0;   // BACK 取消恢复用

/* 前向声明：动态页渲染函数（定义在 render_page 之后，先声明避免隐式声明不兼容） */
static void render_six_axis_page(void);
static void render_attitude_page(void);

/* 动态页(DATA/ATT)"整页重画"标志：进入页面时置1，静态文字画完后清0 */
static uint8_t page_full = 1;

//AI
static void ApplyParams(void){
   NEUQ_Sensor_SetAlpha(Params.alpha);
}

//AI      屏幕渲染：列表页 + 编辑页 + 动态页 
static void render_page(void){
    const MenuConfig *info = page->info;

    /* 动态数据页整页由专用渲染函数处理（自带 100ms 节拍），提前返回，避免双重清屏 */
    if (page == &nodes[MENU_DATA]){
        render_six_axis_page();
        return;
    }
    /* 姿态页：同样整页由专用渲染函数处理，实时刷新 roll/pitch/yaw */
    if (page == &nodes[MENU_ATT]){
        render_attitude_page();
        return;
    }

    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(0, 0, (char*)info->name);
    if (editing == 1){
        /* 编辑页：显示参数名与当前值，如 "Alpha: 50" */
        char buf[24];
        sprintf(buf, "%s: %d", selected->info->name, edit_value);
        NEUQ_LCD200_DisplayString(0, 40, buf);
    } else {
        /* 列表页：遍历 page 的子节点（环形链），选中项加标记 */
        MenuNode *p = page->child;
        if (p != NULL){
            uint16_t y = 20;
            do{
                char buf[24];
                if (p == selected) sprintf(buf, "> %s", p->info->name);
                else               sprintf(buf, "  %s", p->info->name);
                NEUQ_LCD200_DisplayString(0, y, buf);
                y += 20;
                p = p->next;
            } while (p != page->child);
        }
    }
}

/* 六轴数据显示页：静态标签只在进入页面时画一次，数值区每100ms局部刷新，
   避免整屏 Clear+重绘 造成的持续闪烁 */
static void render_six_axis_page(void){
    static uint32_t last = 0;

    if (page_full){
        /* 进入页面：立即整页绘制（跳过节拍检查，让界面马上显示） */
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_DisplayString(0, 0,   "6-Axis Data");
        NEUQ_LCD200_DisplayString(0, 20,  "Acc (g)");
        NEUQ_LCD200_DisplayString(0, 40,  "AX");
        NEUQ_LCD200_DisplayString(0, 60,  "AY");
        NEUQ_LCD200_DisplayString(0, 80,  "AZ");
        NEUQ_LCD200_DisplayString(0, 110, "Gyro (deg/s)");
        NEUQ_LCD200_DisplayString(0, 130, "GX");
        NEUQ_LCD200_DisplayString(0, 150, "GY");
        NEUQ_LCD200_DisplayString(0, 170, "GZ");
        page_full = 0;
        last = NEUQ_sys_tick;              /* 重置节拍基准，避免紧接着又局部刷新一次 */
    } else {
        if ((uint32_t)(NEUQ_sys_tick - last) < 100) return;   /* 100ms 局部刷新 */
        last = NEUQ_sys_tick;
    }

    /* 数值区局部刷新：ClearRect 只清数字区域。默认字体 ASCII_Font24(24px宽)，
       8字符上限=192px，宽度按 192 清，避免上一帧数字右半段残留 */
    NEUQ_LCD200_ClearRect(60, 40,  192, 24);
    NEUQ_LCD200_DisplayFloat(60, 40,  acc_filt[0], 8, 3);
    NEUQ_LCD200_ClearRect(60, 60,  192, 24);
    NEUQ_LCD200_DisplayFloat(60, 60,  acc_filt[1], 8, 3);
    NEUQ_LCD200_ClearRect(60, 80,  192, 24);
    NEUQ_LCD200_DisplayFloat(60, 80,  acc_filt[2], 8, 3);
    NEUQ_LCD200_ClearRect(60, 130, 192, 24);
    NEUQ_LCD200_DisplayFloat(60, 130, gyro_filt[0], 8, 3);
    NEUQ_LCD200_ClearRect(60, 150, 192, 24);
    NEUQ_LCD200_DisplayFloat(60, 150, gyro_filt[1], 8, 3);
    NEUQ_LCD200_ClearRect(60, 170, 192, 24);
    NEUQ_LCD200_DisplayFloat(60, 170, gyro_filt[2], 8, 3);
}

/* 姿态解算显示页：与数据页同构，静态标签一次绘制，roll/pitch/yaw 每100ms局部刷新。
   att 各角为弧度，显示时转成度 */
static void render_attitude_page(void){
    static uint32_t last = 0;

    if (page_full){
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_DisplayString(0, 0,   "Attitude");
        NEUQ_LCD200_DisplayString(0, 40,  "Roll");
        NEUQ_LCD200_DisplayString(0, 70,  "Pitch");
        NEUQ_LCD200_DisplayString(0, 100, "Yaw");
        page_full = 0;
        last = NEUQ_sys_tick;
    } else {
        if ((uint32_t)(NEUQ_sys_tick - last) < 100) return;
        last = NEUQ_sys_tick;
    }

    NEUQ_LCD200_ClearRect(60, 40,  192, 24);
    NEUQ_LCD200_DisplayFloat(60, 40,  att.roll  * ATT_RAD2DEG, 8, 3);
    NEUQ_LCD200_ClearRect(60, 70,  192, 24);
    NEUQ_LCD200_DisplayFloat(60, 70,  att.pitch * ATT_RAD2DEG, 8, 3);
    NEUQ_LCD200_ClearRect(60, 100, 192, 24);
    NEUQ_LCD200_DisplayFloat(60, 100, att.yaw   * ATT_RAD2DEG, 8, 3);
}


//跳转逻辑(AI)
static uint8_t Menu_Update(KEY_EVENT_ENUM key){
    //编辑模式：编辑中 selected 仍是参数节点（页面不变，列表实时显示新值）
    if (editing == 1){
        const MenuConfig *item = selected->info;
        switch (key){
            case KEY_UP:
                edit_value += item->step;
                if (edit_value > item->max){
                edit_value = item->max;
                }
                *item->value = edit_value;
                ApplyParams();          // 修改立即生效
                return 1;
            case KEY_DOWN:
                edit_value -= item->step;
                if (edit_value < item->min){
                    edit_value = item->min;
                }
                *item->value = edit_value;
                ApplyParams();
                return 1;
            case KEY_FORWARD:                       /* 确认保存：写 Flash（第5条） */
                *item->value = edit_value;
                NEUQ_Flash_write_sector(PARAM_SECTOR, (uint8_t *)(void *)&Params, sizeof(Params));
                editing = 0;
                return 1;
            case KEY_BACK:                          /* 取消：恢复进入编辑前的值 */
                *item->value = edit_backup;   // 恢复进入编辑前的备份值
                ApplyParams();
                editing = 0;
                return 1;
            default:
                return 0;
        }
    }
    //浏览模式
    switch (key){
        case KEY_UP:                                /* 同级循环移动（环形链） */
            if (selected != NULL) selected = selected->prev;
            return 1;
        case KEY_DOWN:
            if (selected != NULL) selected = selected->next;
            return 1;
        case KEY_FORWARD:                           /* 进入子页 / 进入编辑 / 进入动态显示页 */
            if (selected != NULL){
                if (selected->child != NULL){
                    page     = selected;            /* 进入子页面 */
                    selected = page->child;
                } else if (selected->info->value != NULL){
                    edit_value = *selected->info->value;
                    edit_backup = *selected->info->value;   
                    editing    = 1;                       /* 进入参数编辑 */
                } else if (selected == &nodes[MENU_DATA] || selected == &nodes[MENU_ATT]){
                    /* 动态显示页（DATA/ATT）：无子无值，FORWARD 进入后整页由
                       render_page 的动态分支渲染，BACK 返回父页 */
                    page     = selected;
                    selected = NULL;
                    page_full = 1;   /* 动态页整页重画一次（画静态标签） */
                }
                /* 其余无子无值节点：FORWARD 无操作 */
            }
            return 1;
        case KEY_BACK:                              /* 返回父页并高亮刚离开的入口 */
            if (page->parent != NULL){
                MenuNode *back = page;
                page     = page->parent;
                selected = back;
            }
            return 1;
        default:
            return 0;
    }
}

void NEUQ_Menu_Init(void){
    Menu_BuildLinks();
    page = &nodes[MENU_MAIN];
    selected = page->child;
    editing = 0;

    if (NEUQ_Flash_read_sector(PARAM_SECTOR, (uint8_t *)(void *)&Params, sizeof(Params)) != NEUQ_FLASH_OK || 
    Params_IsValid() == 0){
        Params.send_axis   = 0;
        Params.send_enable = 0;
        Params.alpha       = 50;
    }

    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    render_page();   /* 上电即渲染主菜单；之后仅在按键/数据节拍时重绘，不再满速清屏 */
}

//AI
void NEUQ_Menu_Task(void){
    uint8_t need_render = 0;
    KEY_EVENT_ENUM key = NEUQ_KEY_Get();
    if (key != KEY_NONE){
        need_render = Menu_Update(key);   /* 按键引起界面变化 → 重绘 */
    }
    /* 动态显示页（DATA/ATT）：100ms 定时刷新；其余页面只在按键时重绘 */
    if (page == &nodes[MENU_DATA] || page == &nodes[MENU_ATT]){
        static uint32_t last = 0;
        if ((uint32_t)(NEUQ_sys_tick - last) >= 100){
            last = NEUQ_sys_tick;
            need_render = 1;
        }
    }
    if (need_render){
        render_page();
    }
}
