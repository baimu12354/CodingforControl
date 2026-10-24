#include "NEUQ_lib_include.h"

volatile uint8_t current = 1;
volatile uint8_t next = 1;

//页面编号
typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_SCHEME_1,
    MENU_SCHEME_2,
    MENU_SCHEME_3,
    MENU_SCHEME_4,
    MENU_COUNT
}MenuID;

//页面信息配置
typedef struct{
    const char *name;
    MenuID parent_id;     // 父节点编号（全部挂在 MENU_MAIN 下）
    uint8_t scheme;       // 该选项对应的 LED 方案号 1~4
}MenuConfig;

//构建菜单结构
static const MenuConfig MenuTable[MENU_COUNT] = {
    [MENU_MAIN]     = {"Scheme Select",       MENU_NONE,  0},
    [MENU_SCHEME_1] = {"1: 1000-1000",        MENU_MAIN,  1},
    [MENU_SCHEME_2] = {"2: 200-1800",         MENU_MAIN,  2},
    [MENU_SCHEME_3] = {"3: 200-200-200-1400", MENU_MAIN,  3},
    [MENU_SCHEME_4] = {"4: 200-200-600-1000", MENU_MAIN,  4},
};

//链表节点
typedef struct MenuNode{
    const MenuConfig *info;
    struct MenuNode *parent;
    struct MenuNode *child;
    struct MenuNode *prev;
    struct MenuNode *next;
}MenuNode;

static MenuNode nodes[MENU_COUNT];
static MenuNode *page;      //指向当前页面
static MenuNode *selected;  //指向当前选项

//根据菜单表格构建节点链接关系
static void Menu_BuildLinks(void){
    
    //第一步：赋值所有节点的页面信息
    for(uint8_t i = 0; i < MENU_COUNT; i++){
        nodes[i].info = &MenuTable[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = NULL;
        nodes[i].next = NULL;
    }

    //第二步:构建各个节点之间的层级关系 和 同级关系
    for(uint8_t i = 0; i < MENU_COUNT; i++){
        //构建层级关系
        MenuNode *node = &nodes[i];
        MenuNode *parent;
        if(node->info->parent_id == MENU_NONE) continue;
        parent = &nodes[node->info->parent_id];
        node->parent = parent;

        //构建同级关系
        MenuNode *first;
        MenuNode *last;
        first = parent->child;
        if(first == NULL){
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

//跳转逻辑
static uint8_t Menu_Update(KEY_EVENT_ENUM key){
    //浏览模式
    switch(key){
        case KEY_UP_SHORT:
            selected = selected->prev;
            break;
        case KEY_DOWN_SHORT:
            selected = selected->next;
            break;
        case KEY_FORWORD_SHORT:
            next = selected->info->scheme;
            break;
        case KEY_BACK_SHORT:
        default:
            return 0;
    }
    return 1;
}

//显示逻辑
static void Menu_Show(void)
{
    MenuNode *item = page->child;
    char line[32];

    NEUQ_LCD200_Clear();

    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    snprintf(line, sizeof(line), "%s", page->info->name);
    NEUQ_LCD200_DisplayString(8, 8, line);

    // 遍历循环链表（到尾节点 next 回到头时结束）
    if(item != NULL){
        uint16_t y = 48;
        do{
            if(item == selected){                   // 高亮项
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8, y, 320, 24);
            } else {                                // 普通项
                NEUQ_LCD200_SetPenColor(LCD_BLACK);
                NEUQ_LCD200_SetBackColor(LCD_WHITE);
            }
            NEUQ_LCD200_DisplayString(8, y, (char*)item->info->name);
            y += 32;
            item = item->next;
        }while(item != page->child);
    }

    // 底部：当前方案 / 待执行方案（题目要求页面同时显示）
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    snprintf(line, sizeof(line), "Current: %d", current);
    NEUQ_LCD200_DisplayString(8, 184, line);
    snprintf(line, sizeof(line), "Next: %d", next);
    NEUQ_LCD200_DisplayString(8, 216, line);
}

void NEUQ_Menu_Init(void)
{
    Menu_BuildLinks();
    page = &nodes[MENU_MAIN];
    selected = page->child;

    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    Menu_Show();
}

void NEUQ_Menu_Task(void)
{
    KEY_EVENT_ENUM key = NEUQ_Key_EventGet();
    if(Menu_Update(key)){
        Menu_Show();
    }
}

void NEUQ_Menu_Refresh(void)
{
    Menu_Show();
}

void Menu_ShowStatus(void){
    char line[32];
    NEUQ_LCD200_ClearRect(8,184,304,24);
    NEUQ_LCD200_ClearRect(8,216,304,24);
    snprintf(line, sizeof(line), "Current: %d", current);
    NEUQ_LCD200_DisplayString(8, 184, line);
    snprintf(line, sizeof(line), "Next: %d", next);
    NEUQ_LCD200_DisplayString(8, 216, line);
}