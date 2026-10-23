#include "NEUQ_lib_include.h"

extern uint32_t beat;

extern Filter_Param *axis[6];

extern float ax;
extern float ay;
extern float az;
extern float gx;
extern float gy;
extern float gz;

extern float pitch;
extern float roll;
extern float yaw;

// 菜单ID
typedef enum{
    MENU_NONE = -1,  //未选择任何菜单
    MENU_DATA = 0,   //六轴数据显示
    MENU_WAVE,       //波形与滤波设置
    MENU_ATTITUDE,   //姿态解算
    MENU_ACC_X,      //x轴加速度
    MENU_ACC_Y,      //y轴加速度
    MENU_ACC_Z,      //z轴加速度
    MENU_W_X,        //x轴角速度
    MENU_W_Y,        //y轴角速度
    MENU_W_Z,        //z轴角速度
    MENU_WAVE_AXIS,  //波形轴选择
    MENU_WAVE_SEND,  //波形上传开关
    MENU_WAVE_A,     //滤波系数
    MENU_WAVE_SAVE,  //保存到Flash
    MENU_ATT_PITCH,  //俯仰角
    MENU_ATT_ROLL,   //横滚角
    MENU_ATT_YAW,    //偏航角
    ZERO_OFFSET,     //零偏校准
    Springboard,     //跳板，寄存在nodes数组，以用来找到整个数组
} MenuID;

// 节点类型
typedef enum{
    ITEM_PAGE,    //子菜单入口
    ITEM_READ,    //只读数值
    ITEM_VAL,     //可编辑数值
    ITEM_TOGGLE,  //开关（0:关  1:开）
    ITEM_CYCLE,   //选项循环
    ITEM_SAVE,    //保存
    ITEM_OFFSET,  //零篇校准
} ItemType;

typedef struct{
    const char *name;           //当前页面名称
    MenuID parent_id;           //当前页面的父节点编号
    ItemType type;              //节点类型
    float *value;               //当前页面的参数地址（NULL表示该节点不编辑参数）
    float min;                  //调参下限
    float max;                  //调参上限
    float step;                 //调参步长
    const char *unit;           //单位
    char *opts;                 //CYCLE 选项名表
} MenuConfig;  //菜单配置

// CYCLE 选项名表
static const char *axis_opts[6] = {"Ax","Ay","Az","Gx","Gy","Gz"};

uint8_t axis_index = 0;   //选择轴序号
float axis_send = 0;      //是否打开发送（0:未打开  1:已打开）
float axis_a = 0.20f;     //当前轴的参数值
char axis_choose[8];      //当前轴名
uint8_t status;           //零偏校准状态

//链表节点结构
typedef struct MenuNode{
    const MenuConfig *info;   //节点内容
    struct MenuNode *parent;  //层级父节点
    struct MenuNode *child;   //层级子节点
    struct MenuNode *prev;    //同级前节点
    struct MenuNode *next;    //同级后节点
}MenuNode;   //当前节点

static MenuNode nodes[Springboard + 1];   //固定内存存储节点
static MenuNode *page;                   //当前页面
static MenuNode *selected;               //当前选项（selected永远是page的子节点）
static uint8_t editing;                  //0:浏览模式  1:编辑模式
static uint32_t last_refresh;            //只读页面自动刷新节拍

//构建菜单结构
static const MenuConfig MenuTable[Springboard + 1] = {
    [MENU_DATA] = {"Data Show", Springboard, ITEM_PAGE, NULL, 0, 0, 0, NULL, NULL},
    [MENU_WAVE] = {"Wave Settings", Springboard, ITEM_PAGE, NULL, 0, 0, 0, NULL, NULL},
    [MENU_ATTITUDE] = {"Attitude calculation", Springboard, ITEM_PAGE, NULL, 0, 0, 0, NULL, NULL},

    [MENU_ACC_X] = {"x Accel", MENU_DATA, ITEM_READ, &ax, 0, 0, 0, "g", NULL},
    [MENU_ACC_Y] = {"y Accel", MENU_DATA, ITEM_READ, &ay, 0, 0, 0, "g", NULL},
    [MENU_ACC_Z] = {"z Accel", MENU_DATA, ITEM_READ, &az, 0, 0, 0, "g", NULL},
    [MENU_W_X] = {"x AngleSpeed", MENU_DATA, ITEM_READ, &gx, 0, 0, 0, "deg/s", NULL},
    [MENU_W_Y] = {"y AngleSpeed", MENU_DATA, ITEM_READ, &gy, 0, 0, 0, "deg/s", NULL},
    [MENU_W_Z] = {"z AngleSpeed", MENU_DATA, ITEM_READ, &gz, 0, 0, 0, "deg/s", NULL},

    [MENU_WAVE_AXIS] = {"Axis", MENU_WAVE, ITEM_CYCLE, (float*)&axis_index, 0, 0, 0, NULL, axis_choose},
    [MENU_WAVE_SEND] = {"Send", MENU_WAVE, ITEM_TOGGLE, &axis_send, 0, 0, 0, NULL, NULL},
    [MENU_WAVE_A] = {"Coeff", MENU_WAVE, ITEM_VAL, &axis_a, 0.01f, 0.99f, 0.01f, NULL, NULL},
    [MENU_WAVE_SAVE] = {"Save to Flash", MENU_WAVE, ITEM_SAVE, NULL, 0, 0, 0, NULL, NULL},

    [MENU_ATT_PITCH] = {"pitch", MENU_ATTITUDE, ITEM_READ, &pitch, 0, 0, 0, "deg", NULL},
    [MENU_ATT_ROLL] = {"roll", MENU_ATTITUDE, ITEM_READ ,&roll, 0, 0, 0, "deg", NULL},
    [MENU_ATT_YAW] = {"yaw", MENU_ATTITUDE, ITEM_READ, &yaw, 0, 0, 0, "deg", NULL},
    [ZERO_OFFSET] = {"zero offset", MENU_ATTITUDE, ITEM_OFFSET, NULL, NULL, NULL, NULL, NULL},

    [Springboard] = {"Main Menu", MENU_NONE, ITEM_PAGE, NULL, 0, 0, 0, NULL, NULL},
};

//根据菜单表格构建节点链接关系
static void Menu_BuildLinks(void){
    
    //第一步：赋值所有节点的页面信息
    for (uint8_t i = 0; i <= Springboard; i++){
        nodes[i].info = &MenuTable[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = NULL;
        nodes[i].next = NULL;
    }

    //第二步：构建各个节点的层级关系和同级关系
    for (uint8_t i = 0; i < Springboard; i++){
        //构建层级关系
        MenuNode *node = &nodes[i];  //当前节点
        MenuNode *parent;            //当前节点的父节点
        if (node->info->parent_id == MENU_NONE) continue;
        parent = &nodes[node->info->parent_id];
        node->parent = parent;

        //构建同级关系
        MenuNode *first;  //第一个被挂载上的子节点
        MenuNode *last;   //最后一个子节点
        first = parent->child;
        
        //编辑第一子节点
        if (first == NULL){
            parent->child = node;
            node->prev = node;
            node->next = node;
        }
        //插入第二子节点（插入到最后）
        else{
            last = first->prev;
            node->prev = last;
            last->next = node;
            node->next = first;
            first->prev = node;
        }
    }
}

float now;

//跳转逻辑
static uint8_t Menu_Turn(KEY_EVENT key){
    //编辑模式（仅 ITEM_VAL）
    if (editing == 1){
        const MenuConfig *item = selected->info;

        switch (key){
          case KEY_S_UP:
             case KEY_L_UP:
             now += item->step;
             break;
          case KEY_S_DOWN:
             case KEY_L_DOWN:
             now -= item->step;
             break;
          case KEY_S_FORWARD:
             *(item->value) = now;
             A_Send(axis_index, axis_a);
             editing = 0;
             break;
          case KEY_S_BACK:
             editing = 0;
             break;
          default:
             return 0;
        }
        if (now < item->min) now = item->min;
        if (now > item->max) now = item->max;
        return 1;
    }

    //浏览模式
    switch (key){
        case KEY_S_UP:
          selected = selected->prev;
          break;
        case KEY_S_DOWN:
          selected = selected->next;
          break;
        case KEY_S_FORWARD:{
          const MenuConfig *item = selected->info;
          switch (item->type){
            case ITEM_PAGE:
              if (selected->child != NULL){
                page = selected;
                selected = selected->child;
              }
              else return 0;
              break;
            case ITEM_VAL:
              editing = 1;
              now = *(item->value);
              break;
            case ITEM_TOGGLE:
              if (*(item->value) == 0){
                *(item->value) = 1;
              }
              else{
                *(item->value) = 0;
              }
              break;
            case ITEM_CYCLE:
              if ((*(uint8_t*)item->value) < 5){
                *(uint8_t*)item->value = (*(uint8_t*)item->value) + 1;
              }
              else *(uint8_t*)item->value = 0;
              strcpy(item->opts, axis_opts[*(uint8_t*)item->value]);
              axis_a = axis[*(uint8_t*)item->value]->a;
              break;
            case ITEM_SAVE:
              Flash_Send(&(A_X.a),&(A_Y.a),&(A_Z.a),&(G_X.a),&(G_Y.a),&(G_Z.a));
              break;
            case ITEM_OFFSET:
              Zero_offset();
              break;
            default:
              return 0;
          }
          break;
        }
        case KEY_S_BACK:
          if (page->parent == NULL) return 0;
          selected = page;
          page = page->parent;
          break;
        default:
          return 0;
    }
    return 1;
}

// 显示逻辑
static void Menu_Show(void){
    MenuNode *item = page->child;

    NEUQ_LCD200_Clear();
    char line[40];
    uint16_t y = 48;
    snprintf(line, sizeof(line), "%s", page->info->name);
    NEUQ_LCD200_DisplayString(8, 8, line);

    if (item != NULL){
        do {
            char mark = ' ';
            const MenuConfig *info = item->info;

            if (item == selected){
                mark = (editing && info->type == ITEM_VAL) ? '*' : '>';
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8, y, 304, 24);
            }

            float i;
            switch (info->type){
                case ITEM_READ:
                    snprintf(line, sizeof(line), "%c %s", mark,info->name);
                    NEUQ_LCD200_DisplayString(8, y, line);
                    NEUQ_LCD200_DisplayFloat(150, y, *(info->value), 6, 1);
                    snprintf(line, sizeof(line), "%s", info->unit);
                    NEUQ_LCD200_DisplayString(224, y, line);
                    break;
                case ITEM_VAL:
                    i = ((item == selected) && editing) ? now : *(info->value);
                    snprintf(line, sizeof(line), "%c %s", mark, info->name);
                    NEUQ_LCD200_DisplayString(8, y, line);
                    NEUQ_LCD200_DisplayFloat(150, y, i, 6, 2);
                    break;
                case ITEM_TOGGLE:
                    snprintf(line, sizeof(line), "%c %s  %s", mark, info->name,
                             (*(info->value)) ? "ON" : "OFF");
                    NEUQ_LCD200_DisplayString(8, y, line);
                    break;
                case ITEM_CYCLE:
                    snprintf(line, sizeof(line), "%c %s  %s", mark, info->name,
                            info->opts);
                    NEUQ_LCD200_DisplayString(8, y, line);
                    break;
                case ITEM_PAGE:
                    snprintf(line, sizeof(line), "%c %s  >", mark, info->name);
                    NEUQ_LCD200_DisplayString(8, y, line);
                    break;
                case ITEM_OFFSET:
                    snprintf(line, sizeof(line), "%c %s  %s", mark, info->name,
                             status == OFFSET_DONE ? "DONE" : (status == OFFSET_RUNNING ? "Running" : "Pending"));
                    NEUQ_LCD200_DisplayString(8, y, line);
                    break;
                default:
                    snprintf(line, sizeof(line), "%c %s", mark, info->name);
                    NEUQ_LCD200_DisplayString(8, y, line);
                    break;
            }

            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y += 32;
            item = item->next;
        }
        while (item != page->child);
    }
}

// 只读数值刷新
static void Menu_Refresh(void){
    MenuNode *item = page->child;
    if (item == NULL) return;

    uint16_t y = 48;
    do {
        const MenuConfig *info = item->info;
        if (info->type == ITEM_READ){
            if (item == selected){
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
            } else {
                NEUQ_LCD200_SetPenColor(LCD_BLACK);
                NEUQ_LCD200_SetBackColor(LCD_WHITE);
            }
            NEUQ_LCD200_ClearRect(150, y, 72, 24);
            NEUQ_LCD200_DisplayFloat(150, y, *(info->value), 6, 1);
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
        }
        y += 32;
        item = item->next;
    } while (item != page->child);
}

// 当前页面是否含只读选项
static uint8_t page_has_read(void){
    MenuNode *item = page->child;
    if(item == NULL) return 0;
    MenuNode *item_new = item;
    do {
        if(item_new->info->type == ITEM_READ) return 1;
        item_new = item_new->next;
    } while(item_new != item);
    return 0;
}

void Menu_Init(void){
    Menu_BuildLinks();
    page = &nodes[Springboard];
    selected = page->child;
    editing = 0;
    axis_index = 0;
    axis_send = 0;
    now = 0;
    last_refresh = 0;
    strcpy(axis_choose, axis_opts[0]);
    axis_a = axis[0]->a;
    status = OFFSET_PENDING;

    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    Menu_Show();
}

void Menu_Task(void){
    KEY_EVENT key;
    key = Key_EventGet();
    if (Menu_Turn(key)){
        Menu_Show();
    }
    else if (page_has_read() && (beat - last_refresh >= 10)){
        last_refresh = beat;   //beat已在NEUQ_Interrupts.c中定义
        Menu_Refresh();
    }
}
