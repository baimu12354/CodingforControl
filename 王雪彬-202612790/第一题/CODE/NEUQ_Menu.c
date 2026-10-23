#include "NEUQ_lib_include.h"


//页面编号
typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_MODE_1,
    MENU_MODE_2,
    MENU_MODE_3,
    MENU_MODE_4,
    MENU_CNT
}MenuID;

//页面信息配置
typedef struct{
    const char *name;   //当前页面名称
    MenuID parent_id;   //当前页面的父页面编号
    int32_t *value;     //当前页面的参数地址（null表示该页面没有参数）
    int32_t min; 
    int32_t max;
    int32_t step;
} MenuConfig;


static int8_t active_mode = MENU_NONE;   

int8_t NEUQ_Menu_GetActiveMode(void){
    return active_mode;
}


//构建菜单结构
static const MenuConfig MenuTable[MENU_CNT] = {
    [MENU_MAIN] = {"Menu_main", MENU_NONE, NULL,0,0,0},
    [MENU_MODE_1] = {"Mode 1", MENU_MAIN, NULL,0,0,0},
    [MENU_MODE_2] = {"Mode 2", MENU_MAIN, NULL,0,0,0},
    [MENU_MODE_3] = {"Mode 3", MENU_MAIN, NULL,0,0,0},
    [MENU_MODE_4] = {"Mode 4", MENU_MAIN, NULL,0,0,0},
};


//链表节点
typedef struct MenuNode{
    const MenuConfig *info;  //节点内容 
    struct MenuNode *parent;  //父节点
    struct MenuNode *child;   //子节点
    struct MenuNode *prev;    //同级前节点
    struct MenuNode *next;    //同级后节点
} MenuNode;

static MenuNode nodes[MENU_CNT];  //固定内存存储节点数组
static MenuNode *page; //当前页面
static MenuNode *selected; //当前选中项
static uint8_t editing; //0为浏览模式，1为编辑模式
static int32_t edit_value;

static void Menu_BuildLinks(void){
    //1.赋值所有节点页面信息
   for (uint8_t i=0; i<MENU_CNT; i++){
        nodes[i].info = &MenuTable[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = NULL;
        nodes[i].next = NULL;
   }
   //2.建立各个节点的层级与同级关系
   for(uint8_t i=0; i<MENU_CNT; i++){
    //构建层级关系
    MenuNode *node = &nodes[i];
    MenuNode *parent;
    if(node->info->parent_id == MENU_NONE)continue;
    parent = &nodes[node->info->parent_id];
    node->parent = parent;

    //构建同级关系
    MenuNode *first;
    MenuNode *last;
    first = parent->child;
    if (first == NULL){
        parent->child = node;
        node->prev = node;
        node->next = node;
    }else{
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
    //编辑模式
    if(editing==1){
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
            case KEY_FORWARD_SHORT: 
                *item->value= edit_value; 
                editing=0; 
                break; 
            case KEY_BACK_SHORT: 
                editing=0; 
                break; 
            default: 
                return 0; 
        }
        if (edit_value<item->min) edit_value = item->min; 
        if (edit_value>item->max) edit_value = item->max; 
        return 1;
    }
    //浏览模式
    switch (key){ 
        case KEY_UP_SHORT:
            if(selected!=NULL)
            selected = selected->prev;
            break;
        case KEY_DOWN_SHORT:
            if(selected!=NULL)
            selected=selected->next;
            break;
        case KEY_FORWARD_SHORT: 
            if (selected->child != NULL){
            page= selected; 
            selected= selected->child; 
        }else if (selected!=NULL){
            active_mode = (int8_t)(selected - nodes);
        }
            break;
 
        case KEY_BACK_SHORT:{
            if (page->parent== NULL) 
            active_mode = MENU_NONE; 
            return 1;
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
    uint16_t y = 40;

    NEUQ_LCD200_Clear();
    char line[32];
    snprintf(line,sizeof(line),"%s",page->info->name);
    NEUQ_LCD200_DisplayString(8,8,line);
    if(item != NULL){
       do {
         char mark=' ';
         const MenuConfig *info=item->info;
         int8_t idx = (int8_t)(item-nodes);

         if( item==selected ){
            mark = editing?'*':'>';
            NEUQ_LCD200_SetPenColor(LCD_WHITE);
            NEUQ_LCD200_SetBackColor(LCD_BLUE);
            NEUQ_LCD200_ClearRect(8,y,320,24);
         }

         if(idx == active_mode){
            mark = '*';
         }

         if(info->value!=NULL){
            int32_t value = ((item==selected)&&(editing))?edit_value:(*info->value) ;
            snprintf(line,sizeof(line),"%c %s:%ld",mark,info->name,value);
         }else{
            snprintf(line,sizeof(line),"%c %s",mark,info->name);

        }
        NEUQ_LCD200_DisplayString(8,y,line);

        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        y+=32;
        item=item->next;
       }   while(item!=page->child);

    }
    return 1;
}

void NEUQ_Menu_Init(void){

    //构建菜单结构
    Menu_BuildLinks();

    //设置初始页面和选中项
    page = &nodes[MENU_MAIN];
    selected = page->child;
    editing = 0; //浏览模式

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

int8_t NEUQ_Menu_GetPage(void){
    return (int8_t)(page - nodes);  //
}

void NEUQ_Menu_SetActiveMode(int8_t mode){
    active_mode = mode;
    Menu_Show();   // 恢复后立即重绘
}






