#include "NEUQ_lib_include.h"

NEUQ_Menu_Params params;

//页面编号
typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_ACTION,
    MENU_COUNT
}MenuID;

//页面信息配置
typedef struct{
    const char *name;   //名称
    MenuID parent_id;   //父页面
    int32_t *value;     //参数，NULL为不编辑
    int32_t min;
    int32_t max;
    int32_t step;
}MenuConfig;

//菜单结构及参数
static const MenuConfig MenuTable[MENU_COUNT] = {
    [MENU_MAIN] = {"Main", MENU_NONE, NULL, 0, 0, 0},
    [MENU_ACTION] = {"Action", MENU_MAIN, &params.pre, 1, 4, 1}
};

//链（Node）表节点的内容，父层级，子层级，同级前一个，同级后一个
typedef struct MenuNode{
    const MenuConfig *info;
    struct MenuNode *parent;
    struct MenuNode *child;
    struct MenuNode *prev;
    struct MenuNode *next;
}MenuNode;

static MenuNode nodes[MENU_COUNT]; //固定内存分配，存储节点
static MenuNode *page;       //当前节点
static MenuNode *selected;   //选项，永为page子节点
static uint8_t  editing;     //0:浏览 1:编辑

//节点链接关系构建
static void Menu_BuildLinks(void){

    //step1：所有节点页面信息赋值
    for(uint8_t i=0;i<MENU_COUNT;i++){
        nodes[i].info =&MenuTable[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = NULL;
        nodes[i].next = NULL;
    }

    //step2：构建各节点层级&同级关系
    for(uint8_t i=0;i<MENU_COUNT;i++){

        //层级关系
        MenuNode *node = &nodes[i];
        MenuNode *parent;
        if (node->info->parent_id == MENU_NONE) continue;
        parent = &nodes[node->info->parent_id];
        node->parent = parent;

        //同级关系
        MenuNode *first;
        MenuNode *last;
        first = parent->child;
        if (first ==NULL){
            parent->child = node;
            node->prev = node;
            node ->next = node;
        }else{
            last = first->prev;
            node->prev = last;
            last->next = node;
            node->next = first;
            first->prev = node;
        }
    }

}

int32_t edit_value = 0;

//跳转逻辑
static uint8_t Menu_Update(KEY_EVENT_ENUM key){
    //编辑模式
    if (editing == 1){
        const MenuConfig *item = selected->info;

        switch(key){
            case KEY_UP_SHORT:
            case KEY_UP_LONG:
                edit_value += item->step;
                break;
            case KEY_DOWN_SHORT:
            case KEY_DOWN_LONG:
                edit_value -= item->step;
                break;
            case KEY_DONE_SHORT:
                *item->value = edit_value;
                editing = 0;
                break;
            case KEY_ABORT_SHORT:
                editing = 0;
                break;
            default:
                return 0;
                break;
        }
        if (edit_value < item->min) edit_value = item->min;
        if (edit_value > item->max) edit_value = item->max;
        return 1;
    }

    //浏览模式
    switch (key){
        case KEY_UP_SHORT:
            selected = selected->prev;
            break;
        case KEY_DOWN_SHORT:
            selected = selected->next;
            break;
        case KEY_DONE_SHORT:
            if (selected->child != NULL){
                page = selected;
                selected = selected->child;
            }else if (selected->info->value != NULL){
                editing = 1;
                edit_value = *selected->info->value;
            }else{
                return 0;
            }
            break;
        case KEY_ABORT_SHORT:
            if (page->parent == NULL) return 0;
            selected = page;
            page = page->parent;
            break;
        default:
            return 0;
    }
    return 1;
}

static uint8_t Menu_Show(void){
    MenuNode *item = page->child;

    NEUQ_LCD200_Clear();
    // 右上角显示当前 action 值
    char current[16];
    snprintf(current, sizeof(current), "Current:%ld", (long)params.pre);
    NEUQ_LCD200_DisplayString(200, 8, current);

    char line[32];
    uint16_t y = 48;
    snprintf(line,sizeof(line),"%s",page->info->name);
    NEUQ_LCD200_DisplayString(8,8,line);

    if (item != NULL){
        do{
            char mark = ' ';
            const MenuConfig *info = item->info;

            if (item == selected){
                mark = editing?'*':'>';
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8, y, 320, 24);
            }
            if (info->value != NULL){
                int32_t value = ((item == selected) && (editing)) ? edit_value : (*info->value);
                snprintf (line,sizeof(line),"%c %s:%ld",mark,info->name,value);
            }else{
                snprintf (line,sizeof(line),"%c %s",mark,info->name);
            }
            NEUQ_LCD200_DisplayString(8,y,line);

            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y+=32;
            item = item->next;  
        } while (item!=page->child);
    }
}

void NEUQ_Menu_Init(void){
    params.action = 1;
    params.pre = 1;
    Menu_BuildLinks();
    page = &nodes[MENU_MAIN];
    selected = page->child;
    editing = 0;
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    Menu_Show();
}

void NEUQ_Menu_Task(void){
    KEY_EVENT_ENUM key;
    key = NEUQ_KEY_EventGet();
    if (Menu_Update(key)){
        Menu_Show();
    }
}
