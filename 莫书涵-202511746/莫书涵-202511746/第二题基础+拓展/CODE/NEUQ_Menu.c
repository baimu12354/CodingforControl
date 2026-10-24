#include "NEUQ_lib_include.h"

NEUQ_Menu_Params Pragma;

//页面编号
typedef enum{
    MENU_NONE = -1,

    //一级页面
    MENU_MAIN = 0,               //总菜单

    //二级页面
    MENU_6_AXIS_DISPLAY,         //六轴数据显示
    MENU_WAVE_FILTER,            //波形与滤波设置
    MENU_ATTITUDE,               //姿态解算

    //三级页面(六轴)
    MENU_XA, MENU_YA, MENU_ZA,
    MENU_XG, MENU_YG, MENU_ZG,

    //六轴页面下选项
    MENU_XA_SEND, MENU_XA_ALPHA,
    MENU_YA_SEND, MENU_YA_ALPHA,
    MENU_ZA_SEND, MENU_ZA_ALPHA,
    MENU_XG_SEND, MENU_XG_ALPHA,
    MENU_YG_SEND, MENU_YG_ALPHA,
    MENU_ZG_SEND, MENU_ZG_ALPHA,

    //计数项
    MENU_COUNT
}MenuID;

//页面信息配置
typedef struct{
    const char *name;            //当前页面名称
    MenuID parent_id;            //当前页面的父节点编号
    int32_t *value;              //当前页面的参数地址（NULL表示该节点不编辑参数）
    int32_t min;                 //参数最小值
    int32_t max;                 //参数最大值
    int32_t step;                //每次调整的幅度
    NEUQ_Menu_Action action;     //当前节点的动作
}MenuConfig;

//static表示变量存在周期为全局，const表示变量不能被修改
//构建菜单结构（菜单由页面组成）
static const MenuConfig MenuTable[MENU_COUNT] = {
    //一级页面
    [MENU_MAIN] =           {"Main menu",       MENU_NONE,NULL,0,0,0,ACTION_NONE},
    //二级页面
    [MENU_6_AXIS_DISPLAY] = {"6-Axis Display",  MENU_MAIN,NULL,0,0,0,ACTION_NONE},
    [MENU_WAVE_FILTER] =    {"Wave & Filter",   MENU_MAIN,NULL,0,0,0,ACTION_NONE},
    [MENU_ATTITUDE] =       {"Attitude",        MENU_MAIN,NULL,0,0,0,ACTION_NONE},
    //三级页面(六轴)
    [MENU_XA] = {"Xa",MENU_WAVE_FILTER,NULL,0,0,0,ACTION_NONE},
    [MENU_YA] = {"Ya",MENU_WAVE_FILTER,NULL,0,0,0,ACTION_NONE},
    [MENU_ZA] = {"Za",MENU_WAVE_FILTER,NULL,0,0,0,ACTION_NONE},
    [MENU_XG] = {"Xg",MENU_WAVE_FILTER,NULL,0,0,0,ACTION_NONE},
    [MENU_YG] = {"Yg",MENU_WAVE_FILTER,NULL,0,0,0,ACTION_NONE},
    [MENU_ZG] = {"Zg",MENU_WAVE_FILTER,NULL,0,0,0,ACTION_NONE},
    //六轴下面的选项
    [MENU_XA_SEND]  = {"Send", MENU_XA,NULL,                  0,0,0,ACTION_SEND_XA},
    [MENU_XA_ALPHA] = {"Alpha",MENU_XA,&Pragma.alpha_1000[0],50,990,10,ACTION_NONE},

    [MENU_YA_SEND]  = {"Send", MENU_YA,NULL,                  0,0,0,ACTION_SEND_YA},
    [MENU_YA_ALPHA] = {"Alpha",MENU_YA,&Pragma.alpha_1000[1],50,990,10,ACTION_NONE},

    [MENU_ZA_SEND]  = {"Send", MENU_ZA,NULL,                  0,0,0,ACTION_SEND_ZA},
    [MENU_ZA_ALPHA] = {"Alpha",MENU_ZA,&Pragma.alpha_1000[2],50,990,10,ACTION_NONE},

    [MENU_XG_SEND]  = {"Send", MENU_XG,NULL,                  0,0,0,ACTION_SEND_XG},
    [MENU_XG_ALPHA] = {"Alpha",MENU_XG,&Pragma.alpha_1000[3],50,990,10,ACTION_NONE},

    [MENU_YG_SEND]  = {"Send", MENU_YG,NULL,                  0,0,0,ACTION_SEND_YG},
    [MENU_YG_ALPHA] = {"Alpha",MENU_YG,&Pragma.alpha_1000[4],50,990,10,ACTION_NONE},

    [MENU_ZG_SEND]  = {"Send", MENU_ZG,NULL,                  0,0,0,ACTION_SEND_ZG},
    [MENU_ZG_ALPHA] = {"Alpha",MENU_ZG,&Pragma.alpha_1000[5],50,990,10,ACTION_NONE},
};

//链表节点
typedef struct MenuNode{
    const MenuConfig *info;     //节点内容
    struct MenuNode *parent;    //层级父节点
    struct MenuNode *child;     //层级子节点
    struct MenuNode *prev;      //同级前节点
    struct MenuNode *next;      //同级后节点
}MenuNode;

static MenuNode nodes[MENU_COUNT];     //固定内存存储节点
static MenuNode *page;                 //指向当前页面
static MenuNode *selected;             //指向当前选项（selected 永远是page 的子节点）
static uint8_t editing;                //0:浏览模式  1:编辑模式
static MenuID page_id;                 //用于给显示逻辑提供外部接口

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
        MenuID parent_id = node->info->parent_id;
        if(parent_id == MENU_NONE) continue;
        MenuNode *parent = &nodes[parent_id];
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

//动作逻辑，用于更改发送轴和更新发送请求标志位
static void Menu_DoAction(const MenuConfig *item){
    int32_t axis = item->action;
    if(axis < ACTION_SEND_XA || axis > ACTION_SEND_ZG) return;

    if((Pragma.send_require) && (Pragma.send_axis == axis)){
        Pragma.send_require = 0;
    }else{
        __disable_irq();//AI说这两个量要被整体观察，为了防止两个赋值中间有中断，加上这个暂时屏蔽中断
        Pragma.send_axis = axis;
        Pragma.send_require = 1;
        __enable_irq();
    }
}

int32_t edit_value = 0;

//跳转逻辑
static uint8_t Menu_Update(KEY_EVENT_ENUM key){
    //编辑模式
    if(editing == 1){
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
            case KEY_FORWORD_SHORT:
                *item->value = edit_value;
                editing = 0;
                Pragma.save_require = 1;
                break;
            case KEY_BACK_SHORT:
                editing = 0;
                break;
            default:
                return 0;
        }
        if(edit_value < item->min) edit_value = item->min;
        if(edit_value > item->max) edit_value = item->max;
        return 1;
    }

    //浏览模式
    if(selected == NULL){//六轴显示页面
        if(key == KEY_BACK_SHORT){
            selected = page;
            page = page->parent;
            page_id = (MenuID)(page - &nodes[0]);
            return 1;
        }
        return 0;
    }

    switch(key){
        case KEY_UP_SHORT:
            selected = selected->prev;
            break;
        case KEY_DOWN_SHORT:
            selected = selected->next;
            break;
        case KEY_FORWORD_SHORT:
            if(selected->child != NULL){
                page = selected;               //页指针指向选择的项
                selected = selected->child;    //项指针指向之前的项的子节点
            } else if (selected->info->value != NULL){
                editing = 1;
                edit_value = *selected->info->value;
            } else if (selected->info->action != ACTION_NONE){
                Menu_DoAction(selected->info);
            } else{//六轴数据显示界面
                page = selected;
                selected = NULL;
            }
            page_id = (MenuID)(page - &nodes[0]);//计算元素个数差
            break;
        case KEY_BACK_SHORT:
            if(page->parent == NULL) return 0;
            selected = page;
            page = page->parent;
            break;
        default:
            return 0;
    }
    return 1;
}

//显示逻辑
static void Menu_Show(void){
    switch(page_id){//纯显示页面接口
        case MENU_6_AXIS_DISPLAY:
            //六轴数据显示函数接口
            //静态显示部分
            NEUQ_SIX_AXIS_DATA_NameDisplay();
            return;
        case MENU_ATTITUDE:
            //姿态解算函数接口
            NEUQ_ATTITUDE_NameDisplay();
            return;
        default:
            break;   
    }

    MenuNode *item = page->child;

    NEUQ_LCD200_Clear();
    char line[32];
    uint16_t y = 48;
    snprintf(line,sizeof(line),"%s",page->info->name);
    NEUQ_LCD200_DisplayString(8,8,line);

    if(item != NULL){
        do{
            char mark = ' ';
            const MenuConfig *info = item->info;

            if(item == selected){
                mark = editing?'*':'>';
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8,y,232,24);
            }

            if(info->action != ACTION_NONE){
                int32_t axis = info->action;
                snprintf(line,sizeof(line),"%c %s:%s",mark,info->name,
                        (Pragma.send_require && Pragma.send_axis == axis) ? "ON" : "OFF");
            }else if(info->value != NULL){
                int32_t value = ((item == selected) && (editing)) ? edit_value : (*info->value); 
                snprintf(line,sizeof(line),"%c %s:%.3f",mark,info->name,(float)value/1000.0f);
            }else {
                snprintf(line,sizeof(line),"%c %s",mark,info->name);
            }
            NEUQ_LCD200_DisplayString(8,y,line);

            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y+=32;
            item = item->next;
        }while(item != page->child);//走完一圈终止
    }
}

uint8_t NEUQ_Alpha_Save(void){
    uint8_t temp;
    temp = NEUQ_Flash_save(ALPHA_SAVE_ADDRESS,(const uint8_t *)Pragma.alpha_1000,sizeof(Pragma.alpha_1000));
    return temp;
}

void NEUQ_Alpha_load(void){
    NEUQ_Flash_init();
    //赋默认值
    for(uint8_t i = 0; i < 6; i++){
        Pragma.alpha_1000[i] = ALPHA1000_MOREN;
    }
    //Flash读取是否成功
    if(NEUQ_Flash_read(ALPHA_SAVE_ADDRESS,(uint8_t *)Pragma.alpha_1000,sizeof(Pragma.alpha_1000)) != NEUQ_FLASH_OK)return;
    //数据合理性判断
    for(uint8_t i = 0; i < 6; i++){
        if(Pragma.alpha_1000[i] < ALPHA1000_MIN || Pragma.alpha_1000[i] > ALPHA1000_MAX){
            Pragma.alpha_1000[i] = ALPHA1000_MOREN;
        }
    }
}

void NEUQ_Config_Task(void){
    if(Pragma.save_require == 1){
        Pragma.save_require = 0;
        NEUQ_Alpha_Save();
    }
}

void NEUQ_Six_Axis_SendTask(void){
    static uint8_t cnt = 0;//AI说是为了完成节流
    static uint8_t last_axis = 0xFF;
    uint8_t axis;

    if(Pragma.send_require != 1){
        cnt = 0;
        last_axis = 0xFF;
        return;
    }

    axis = Pragma.send_axis;
    if(axis > 5) axis = 0;

    cnt++;
    if(cnt < 4 && axis == last_axis){
        return;
    }
    cnt = 0;
    last_axis = axis;
    NEUQ_SIX_AXIS_DATA_SendtoVOFA((AXIS_NAME)axis);
}

void NEUQ_Menu_Init(void){
    Pragma.send_axis = 0;
    Pragma.send_require = 0;
    Pragma.save_require = 0;

    NEUQ_Alpha_load();

    Menu_BuildLinks();
    page = &nodes[MENU_MAIN];
    selected = page->child;
    editing = 0;
    page_id = MENU_MAIN;

    NEUQ_LCD200_init();
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    Menu_Show();
}

void NEUQ_Menu_Task(void){
    KEY_EVENT_ENUM key;
    key = NEUQ_Key_EventGet();
    int8_t redraw_require;
    redraw_require = Menu_Update(key);       //按键按下，重绘请求置 1；无按键按下的时候置 0

    NEUQ_Config_Task();
    NEUQ_Six_Axis_SendTask();

    static MenuID last_page_id = (MenuID)-2;
    int32_t page_change = (last_page_id != page_id);

    static uint32_t last_time = 0;
    if(page_id == MENU_6_AXIS_DISPLAY || page_id == MENU_ATTITUDE){
        // uint32_t now_time = NEUQ_Get_Timer_Value_ms();
        uint32_t now_time = HAL_GetTick();
        if((now_time - last_time) >= Dynamic_refresh_time){
            last_time = now_time;
            redraw_require = 1;
        }
    }

    if(redraw_require == 0)return;

    // if(page_id != MENU_6_AXIS_DISPLAY){
    //     Menu_Show();
    // } else{//六轴数据显示页
    //     if(page_change){
    //         Menu_Show();
    //     }
    //     //六轴数据动态重绘函数：
    //     NEUQ_SIX_AXIS_DATA_DataDisplay();
    // }

    if(page_id == MENU_6_AXIS_DISPLAY){
        if(page_change){
            Menu_Show();
        }
        NEUQ_SIX_AXIS_DATA_DataDisplay();
    } else if(page_id == MENU_ATTITUDE){
        if(page_change){
            Menu_Show();
        }
        NEUQ_ATTITUDE_DataDisplay();
    } else {
        Menu_Show();
    }
    last_page_id = page_id;
    //NEUQ_Config_Task(); 
}
