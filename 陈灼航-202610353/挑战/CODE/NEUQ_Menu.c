#include "NEUQ_lib_include.h"

NEUQ_Menu_Params params;

//页面信息配置
typedef struct{
    const char *name; //当前页面名称
    MenuID parent_id; //当前页面的父节点编号
    int32_t *value;   //当前页面的参数地址（NULL表示该节点不编辑参数）
    int32_t min;      //编辑参数的最小值
    int32_t max;      //编辑参数的最大值
    int32_t step;     //参数变化的步长
} MenuConfig;

//构建菜单结构
static const MenuConfig MenuTable[MENU_COUNT] = {
    [MENU_MAIN] = {"Function",MENU_NONE,NULL,0,0,0},
    [MENU_DISPLAY] = {"6-Axis Data",MENU_MAIN,NULL,0,0,0},
    [MENU_AX] = {"Ax",MENU_DISPLAY,NULL,0,0,0},
    [MENU_AY] = {"Ay",MENU_DISPLAY,NULL,0,0,0},
    [MENU_AZ] = {"Az",MENU_DISPLAY,NULL,0,0,0},
    [MENU_GX] = {"Gx",MENU_DISPLAY,NULL,0,0,0},
    [MENU_GY] = {"Gy",MENU_DISPLAY,NULL,0,0,0},
    [MENU_GZ] = {"Gz",MENU_DISPLAY,NULL,0,0,0},
    [MENU_WAVE] = {"Wave&Filter",MENU_MAIN,NULL,0,0,0},
    [MENU_AXIS] = {"Axis",MENU_WAVE,&params.send_axis,0,6,1},
    [MENU_SEND] = {"Send",MENU_WAVE,&params.send_enable,0,1,1},
    [MENU_FILTER] = {"Filter",MENU_WAVE,&params.filter_coef,0,100,1},
    [MENU_ATTITUDE] = {"Attitude",MENU_MAIN,NULL,0,0,0},
    [MENU_ROLL] = {"Roll",MENU_ATTITUDE,NULL,0,0,0},
    [MENU_PITCH] = {"Pitch",MENU_ATTITUDE,NULL,0,0,0},
    [MENU_YAW] = {"Yaw",MENU_ATTITUDE,NULL,0,0,0},
};

//链表节点结构
typedef struct MenuNode{
    const MenuConfig *info;  //节点内容
    struct MenuNode *parent; //层级父节点
    struct MenuNode *child;  //层级子节点
    struct MenuNode *prev;   //同级前节点
    struct MenuNode *next;   //同级后节点
} MenuNode;

static MenuNode nodes[MENU_COUNT];  //固定内存存储节点
static MenuNode *page;       //当前页面
static MenuNode *selected;   //当前选项（selected 永远是 page 的子字节）
static uint8_t editing;      //0：浏览模式   1：编辑模式

//根据菜单表格构建节点链接关系
static void Menu_BuildLinks(void){

    //第一步：赋值所有节点的页面信息并清空指针做初始化
    for(uint8_t i=0; i<MENU_COUNT; i++){
        nodes[i].info = &MenuTable[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = NULL;
        nodes[i].next = NULL;
    }

    //第二步：构建各个节点之间的层级关系和同级关系
    for(uint8_t i=0; i<MENU_COUNT; i++){
        //构建层级关系
        MenuNode * node = &nodes[i];
        MenuNode *parent;
        if(node->info->parent_id == MENU_NONE)continue;
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

int32_t edit_value = 0;

//跳转逻辑
static uint8_t Menu_Update(KEY_EVENT_ENUM key){
    //编辑模式
    if(editing == 1){
        const MenuConfig *item = selected->info;

        switch (key){
            case KEY_UP_SHORT:
            case KEY_UP_LONG:
                edit_value += item->step;
                break;
            case KEY_DOWN_SHORT:
            case KEY_DOWN_LONG:
                edit_value -= item->step;
                break;
            case KEY_FORWARD_SHORT:
                *item->value = edit_value;
                editing = 0;
                break;
            case KEY_BACK_SHORT:
                editing = 0;
                break;
            default:
                return 0;
        }
        if(edit_value < item->min)edit_value = item->min;
        if(edit_value > item->max)edit_value = item->max;
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
        case KEY_FORWARD_SHORT:
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
       case KEY_BACK_SHORT:{
            if (page->parent == NULL) return 0;
            selected = page;
            page = page->parent;
            break; 
       }
       default:{
            return 0;
       }
    }
    return 1;
}

//显示逻辑
static uint8_t Menu_Show(void){
    MenuNode *item = page->child;

    NEUQ_LCD200_Clear();
    char line[32];
    uint16_t y = 48;
    snprintf(line,sizeof(line),"%s",page->info->name);
    NEUQ_LCD200_DisplayString(8,8,line);

    if (item != NULL){
        do{
            char mark = ' ';
            const MenuConfig *info = item->info;

            if(item == selected){
                mark = editing?'*':'>';
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8,y,320,24);
            }

            if(info->value != NULL){
                int32_t value = ((item == selected)&&(editing))?edit_value : (*info->value);
                snprintf(line,sizeof(line),"%c %s:%ld",mark,info->name,value);
            }else {
                snprintf(line,sizeof(line),"%c %s",mark,info->name);
            }
            NEUQ_LCD200_DisplayString(8,y,line);

            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y+=32;
            item = item->next;
        }while(item!=page->child); 
    }
}

void NEUQ_Menu_Init(void){
    params.send_axis = 0;
    params.send_enable = 0;
    params.filter_coef = 30; //滤波默认值,a = 0.3
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
    key = NEUQ_Key_EventGet();
    if(Menu_Update(key)){
        Menu_Show();
    }
}

MenuID NEUQ_Menu_GetPage(void){
    return (MenuID)(page - nodes);
}