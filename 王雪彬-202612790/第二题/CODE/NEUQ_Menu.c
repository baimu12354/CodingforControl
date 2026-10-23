#include "NEUQ_lib_include.h"


NEUQ_Menu_Params params;
// 掉电保存参数
#define PARAM_MAGIC  0x4E455553UL   // "NEUQ"

typedef struct{
    uint32_t magic;
    int32_t wave_send;
    int32_t wave_axis;
    int32_t wave_alpha_acc;
    int32_t wave_alpha_gyro;
} ParamStore;


static void Menu_Params_Load(void);
static void Menu_Params_Save(void);



//页面编号
typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_SIX_DATA,
    MENU_WAVE_FILTER,
    MENU_WAVE_SEND,
    MENU_WAVE_AXIS,
    MENU_WAVE_ALPHA_ACC,
    MENU_WAVE_ALPHA_GYRO,
    MENU_A_ESTIMATION,
    MENU_A_X,
    MENU_A_Y,
    MENU_A_Z,
    MENU_G_X,
    MENU_G_Y,
    MENU_G_Z,
    MENU_ROLL,
    MENU_PITCH,
    MENU_YAW,
    MENU_CNT
}MenuID;

//页面信息配置
typedef struct{
    const char *name;   //当前页面名称
    MenuID parent_id;   //当前  页面的父页面编号
    int32_t *value;     //当前页面的参数地址（null表示该页面没有参数）
    int32_t min; 
    int32_t max;
    int32_t step;
    uint8_t decimals;  //小数位数
    const char *const *labels;   // 枚举标签(NULL=显示数字)
} MenuConfig;



static const char *const axis_labels[6] = {"AX","AY","AZ","GX","GY","GZ"};

//构建菜单结构
static const MenuConfig MenuTable[MENU_CNT] = {
    [MENU_MAIN] = {"Menu_main", MENU_NONE, NULL,0,0,0,0,NULL},
    [MENU_SIX_DATA] = {"Six Data", MENU_MAIN, NULL,0,0,0,0,NULL},
    [MENU_WAVE_FILTER] = {"Wave Filter", MENU_MAIN, NULL,0,0,0,0,NULL},
    [MENU_WAVE_SEND] = {"Send", MENU_WAVE_FILTER, &params.wave_send, 0, 1, 1, 0,NULL},
    [MENU_WAVE_AXIS] = {"Axis", MENU_WAVE_FILTER, &params.wave_axis, 0, 5, 1, 0,axis_labels},
    [MENU_WAVE_ALPHA_ACC] = {"A_Alpha", MENU_WAVE_FILTER, &params.wave_alpha_acc, 0, 100, 1, 2,NULL},
    [MENU_WAVE_ALPHA_GYRO] = {"G_Alpha", MENU_WAVE_FILTER, &params.wave_alpha_gyro, 0, 100, 1, 2,NULL},
    [MENU_A_ESTIMATION] = {"A_Estimation", MENU_MAIN, NULL,0,0,0,0,NULL},
    [MENU_A_X] = {"A_X(g)", MENU_SIX_DATA, &params.a_x,0,0,0,3,NULL},
    [MENU_A_Y] = {"A_Y(g)", MENU_SIX_DATA, &params.a_y,0,0,0,3,NULL},
    [MENU_A_Z] = {"A_Z(g)", MENU_SIX_DATA, &params.a_z,0,0,0,3,NULL},
    [MENU_G_X] = {"G_X(d/s)", MENU_SIX_DATA, &params.g_x,0,0,0,1,NULL},
    [MENU_G_Y] = {"G_Y(d/s)", MENU_SIX_DATA, &params.g_y,0,0,0,1,NULL},
    [MENU_G_Z] = {"G_Z(d/s)", MENU_SIX_DATA, &params.g_z,0,0,0,1,NULL},
    [MENU_ROLL] = {"Roll(deg)", MENU_A_ESTIMATION, &params.roll,0,0,0,2,NULL},
    [MENU_PITCH] = {"Pitch(deg)", MENU_A_ESTIMATION, &params.pitch,0,0,0,2,NULL},
    [MENU_YAW] = {"Yaw(deg)", MENU_A_ESTIMATION, &params.yaw,0,0,0,2,NULL},
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
                Menu_Params_Save();//保存参数到Flash
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
            selected = selected->prev; 
            break; 
        case KEY_DOWN_SHORT: 
            selected=selected->next; 
            break; 
        case KEY_FORWARD_SHORT: 
            if (selected->child != NULL){
            page= selected; 
            selected= selected->child; 
        }else if (selected->info->value != NULL && selected->info->step > 0){
            editing=1;
            edit_value=*selected->info->value;
        }else{
            return 0;
        }
            break;
 
        case KEY_BACK_SHORT:{
            if (page->parent== NULL) return 0;
            selected =page;
            page=page->parent;
            break;
        }
        default:{
            return 0;
        }

    }
    return 1;
}

static void format_fixed(char *buf, int32_t val, uint8_t dec){
    int32_t div = 1;
    for(uint8_t i=0;i<dec;i++) div *= 10;

    uint32_t u = (val < 0) ? (uint32_t)(-val) : (uint32_t)val;
    uint32_t ip = u / div;   // 整数部分
    uint32_t fp = u % div;   // 小数部分

    char frac[16];
    for(int i = dec-1; i >= 0; i--){
        frac[i] = (char)('0' + fp % 10);
        fp /= 10;
    }
    frac[dec] = '\0';

    snprintf(buf, 32, "%s%lu.%s", (val<0)?"-":"", (unsigned long)ip, frac);
}

static void Menu_DrawItem(uint16_t y, const MenuConfig *info, char mark, int32_t value){
    char line[32];
    if(info->value != NULL){
        if(info->labels != NULL){
            snprintf(line, sizeof(line), "%c %-8s:%7s", mark, info->name, info->labels[value]);
        }else if(info->decimals > 0){
            char num[24];
            format_fixed(num, value, info->decimals);
            snprintf(line, sizeof(line), "%c %-8s:%7s", mark, info->name, num);
        }else{
            snprintf(line, sizeof(line), "%c %-8s:%7ld", mark, info->name, value);
        }
    }else{
        snprintf(line, sizeof(line), "%c %s", mark, info->name);
    }
    NEUQ_LCD200_DisplayString(8, y, line);
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

         if( item==selected ){
            mark = editing?'*':'>';
            NEUQ_LCD200_SetPenColor(LCD_WHITE);
            NEUQ_LCD200_SetBackColor(LCD_BLUE);
            NEUQ_LCD200_ClearRect(8,y,320,24);
         }
         int32_t value = ((item==selected)&&(editing)) ? edit_value
                                                       : (info->value ? *info->value : 0);
         Menu_DrawItem(y, info, mark, value);

        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        y+=32;
        item=item->next;
       }   while(item!=page->child);

    }
    return 1;
}


static void Menu_Params_Load(void){
    ParamStore store;
    if(NEUQ_Flash_read_sector(NEUQ_FLASH_LAST_SECTOR_ID, (uint8_t*)&store, sizeof(store)) != NEUQ_FLASH_OK){
        return;                          // 读失败，用默认值
    }
    if(store.magic != PARAM_MAGIC){
        return;                          // 无有效记录，用默认值
    }
    params.wave_send  = store.wave_send;
    params.wave_axis  = store.wave_axis;
    params.wave_alpha_acc = store.wave_alpha_acc;
    params.wave_alpha_gyro = store.wave_alpha_gyro;
}

// 把当前波形参数写入Flash，供断电重启后恢复
static void Menu_Params_Save(void){
    ParamStore store;
    store.magic      = PARAM_MAGIC;
    store.wave_send  = params.wave_send;
    store.wave_axis  = params.wave_axis;
    store.wave_alpha_acc = params.wave_alpha_acc;
    store.wave_alpha_gyro = params.wave_alpha_gyro;
    NEUQ_Flash_write_sector(NEUQ_FLASH_LAST_SECTOR_ID, (uint8_t*)&store, sizeof(store));
}


void NEUQ_Menu_Init(void){
    NEUQ_Flash_init(); 
    //初始化参数
    params.a_x = 0;
    params.a_y = 0;
    params.a_z = 0;
    params.g_x = 0;
    params.g_y = 0;
    params.g_z = 0;
    params.wave_send = 0;    // 默认关闭发送
    params.wave_axis = 0;    // 默认选 AX
    params.wave_alpha_acc = 20;  // 默认加速度滤波系数 0.20 
    params.wave_alpha_gyro = 9; // 默认角速度滤波系数 0.09 
    params.roll  = 0;        // 姿态角初值 0°
    params.pitch = 0;
    params.yaw   = 0;

    Menu_Params_Load();              // 有有效记录就覆盖默认值


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

// 实时刷新
void NEUQ_Menu_Refresh(void){
    if(editing) return;
    if(page->child == NULL) return;
    if(page->child->info->value == NULL) return;

    MenuNode *item = page->child;
    uint16_t y = 40;
    do {
        const MenuConfig *info = item->info;
        if(info->value != NULL){
            char mark = (item == selected) ? '>' : ' ';
            if(item == selected){              
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
            }
            Menu_DrawItem(y, info, mark, *info->value);
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
        }
        y += 32;
        item = item->next;
    } while(item != page->child);
}







