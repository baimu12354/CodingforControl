//建立二级菜单：
// 一级功能选择页提供三个入口，分别进入“六轴数据显示”“波形与滤波设置”“姿态解算”二级页面。
// 支持光标移动、进入、返回及参数编辑。 

#include "menu.h"
int8_t Menu_cnt = 8;  
float Transmission_switch_cnt = 0;
float Select_transmission_cnt = 0;

//规定默认值（"无有效记录时采用合理默认值"就指这里）
#define KALMAN_DEFAULT_Q  0.1f
#define KALMAN_DEFAULT_R  30.0f
float Kalman_q_cnt = KALMAN_DEFAULT_Q;
float Kalman_r_cnt = KALMAN_DEFAULT_R;

MENU_Six_axis_params params;
Attitude_params Aparams;



//页面信息配置
typedef struct{
    const char *name;   //当前页面名称
    Menu_ID parent_id;   //当前页面父节点
    float *value;     //当前页面参数（NULL表示该节点不编辑参数）
    float min;
    float max;
    float step;
}MenuConfig;

//构建链表
typedef struct MenuNode{
    const MenuConfig *info;     //节点内容
    struct MenuNode *parent;    //父节点
    struct MenuNode *child;     //子节点
    struct MenuNode *prev;      //同级前节点
    struct MenuNode *next;      //同级后节点
}MenuNode;

static MenuNode nodes[MENU_COUNT];  //固定内存存储节点
static MenuNode *page;              //当前页面
static MenuNode *selected;          //当前选项，是page的子节点
static uint8_t editing;             //0:浏览模式 1：编辑模式

typedef enum{
    Transmission_ax = 0,
    Transmission_ay,
    Transmission_az,
    Transmission_gx,
    Transmission_gy,
    Transmission_gz,
} Select_Transmission;


//构建菜单结构（菜单由界面组成）
//[] = {"",,,,,},
static const MenuConfig Menutable[MENU_COUNT] = {
    [MENU_MAIN] = {"Main menu",MENU_NONE,NULL,0,0,0},
    [MENU_Six_axis] = {"Six axis Data Display",MENU_MAIN,NULL,0,0,0},
    [MENU_Waveform] = {"Waveform and Filter",MENU_MAIN,NULL,0,0,0},
    [MENU_Attitude] = {"Attitude Calculation",MENU_MAIN,NULL,0,0,0},
    
    [MENU_ax] = {"ax/g  ",MENU_Six_axis,&params.MENU_ax,0,0,0},
    [MENU_ay] = {"ay/g  ",MENU_Six_axis,&params.MENU_ay,0,0,0},
    [MENU_az] = {"az/g  ",MENU_Six_axis,&params.MENU_az,0,0,0},
    [MENU_gx] = {"gx/deg",MENU_Six_axis,&params.MENU_gx,0,0,0},
    [MENU_gy] = {"gy/deg",MENU_Six_axis,&params.MENU_gy,0,0,0},
    [MENU_gz] = {"gz/deg",MENU_Six_axis,&params.MENU_gz,0,0,0},
    [MENU_Select_transmission] = {"Transmission Set:",MENU_Waveform,&Select_transmission_cnt,0,5,1},    //选择数据轴
    [MENU_Transmission_switch] = {"Switch:",MENU_Waveform,&Transmission_switch_cnt,0,1,1},              //发送开关
    [MENU_Kalman_q] = {"Filter q:",MENU_Waveform,&Kalman_q_cnt,0.0001f,10.0f,0.01f},    // 卡尔曼q
    [MENU_Kalman_r] = {"Filter r:",MENU_Waveform,&Kalman_r_cnt,0.01f,5000.0f,1.0f},     // 卡尔曼r
    [MENU_roll] = {"roll /deg",MENU_Attitude,&Aparams.Attitude_roll,0,0,0},
    [MENU_pitch] = {"pitch/deg",MENU_Attitude,&Aparams.Attitude_pitch,0,0,0},
    [MENU_yaw] = {"yaw  /deg",MENU_Attitude,&Aparams.Attitude_yaw,0,0,0},
    
};

//判断当前项是否需要写入Flash
static uint8_t IsFlashParam(const MenuConfig *item){
    return item == &Menutable[MENU_Select_transmission] ||
           item == &Menutable[MENU_Transmission_switch] ||
           item == &Menutable[MENU_Kalman_q] ||
           item == &Menutable[MENU_Kalman_r];
}

//自动建链
static void Menu_BuildLinks(void){
    //自动建链  1.配置节点信息
    for(uint8_t i = 0; i<MENU_COUNT; i++){
        nodes[i].info = &Menutable[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].next = NULL;
        nodes[i].prev = NULL;
    }
    //自动建链  2.配置节点间关系
    for(uint8_t i = 0; i <MENU_COUNT; i++){
        MenuNode *node = &nodes[i];
        MenuNode *parent;
        //配置层级关系
        if(node->info->parent_id == MENU_NONE){
            continue;
        }
        parent = &nodes[node->info->parent_id];
        node->parent = parent;
        //配置同级关系
        MenuNode *first;
        MenuNode *last;
        first = parent->child;
        if(first == NULL){
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
float edit_value = 0;
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
            case KEY_FORWARD_LONG:            // 长按确认同样生效
                *item->value = edit_value;    // 确认：写入
                editing = 0;
                if(IsFlashParam(item)){
                    Config_Save();
                    Filter_ApplyParams(Kalman_q_cnt, Kalman_r_cnt);
                }
                break;
            case KEY_BACK_SHORT:
            case KEY_BACK_LONG:               // 编辑中按返回：放弃修改并一步返回上级
                editing = 0;
                if(page->parent != NULL){
                    selected = page;
                    page = page->parent;
                }
                break;
            default:
                return 0;
        }
        if (edit_value < item->min) edit_value = item->min;
        if (edit_value > item->max) edit_value = item->max;
        return 1;
    }
    //浏览模式
    switch (key){
        case KEY_UP_SHORT:
        case KEY_UP_LONG:
            selected = selected->prev;        // UP 向上
            break;
        case KEY_DOWN_SHORT:
        case KEY_DOWN_LONG:
            selected = selected->next;        // DOWN 向下
            break;
        case KEY_FORWARD_SHORT:
        case KEY_FORWARD_LONG:                // 长按进入同样生效
            if(selected->child != NULL){
                page = selected;
                selected = selected->child;
            }else if(selected->info->value != NULL){
                editing = 1;
                edit_value = *selected->info->value;
            }else{
                return 0;
            }
            break;
        case KEY_BACK_SHORT:
        case KEY_BACK_LONG:                   // 长按返回同样生效
            if(page->parent == NULL) return 0;
            selected = page;
            page = page->parent;
            break;
        default:
            return 0;
    }
    return 1;
}


//显示逻辑（void，消掉#940警告）
static void Menu_Show(void){
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
                NEUQ_LCD200_ClearRect(8,y,320,24);
            }

            if (info->value != NULL){
                float value = ((item == selected) && (editing)) ? edit_value : (*info->value);
                snprintf(line,sizeof(line),"%c %s:%f",mark,info->name,value);
            }else {
                snprintf(line,sizeof(line),"%c %s",mark,info->name);
            }
            NEUQ_LCD200_DisplayString(8,y,line);

            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y += 32;
            item = item->next;
        }while (item != page->child);
    }
}

void Menu_Init(void){
    params.MENU_ax = 0;
    params.MENU_ay = 0;
    params.MENU_az = 0;
    params.MENU_gx = 0;
    params.MENU_gy = 0;
    params.MENU_gz = 0;
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

void Menu_Task(void){
    KEY_EVENT_ENUM key;
    key = Key_EventGet();
    if(Menu_Update(key)){
        Menu_Show();                     // 按键引起变化才全屏重绘
    }
    //六轴页面：每100ms只刷新数值区域
    if((page == &nodes[MENU_Six_axis]) && (Menu_cnt >= 10)){
        Menu_cnt = 0;
        SixAxis_Refresh();
    }
    //姿态解算：每100ms只刷新数值区域
    if((page == &nodes[MENU_Attitude]) && (Menu_cnt >= 10)){
        Menu_cnt = 0;
        Attitude_Refresh();
    }
}

float Menu_Getvalue(Menu_ID id){
    const MenuConfig *info = nodes[id].info;
    if((info != NULL) && (info->value != NULL)){
        return *info->value;
    }else {
        return 0;
    }
}



void SixAxis_Refresh(void){
    MenuNode *item = page->child;
    uint16_t y = 48;
    uint8_t r = 0;
    do{
        if(r >= 6) break;                    // 只画6行数据

        if(item == selected){                // 当前选中行：保持蓝底白字
            NEUQ_LCD200_SetPenColor(LCD_WHITE);
            NEUQ_LCD200_SetBackColor(LCD_BLUE);
        }else{                               // 其他行：白底黑字
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
        }

        NEUQ_LCD200_ClearRect(69+40, y, 200, 24);          // 按当前背景色清数值区
        float value = ((item == selected) && (editing)) ? edit_value : (*item->info->value);
        NEUQ_LCD200_DisplayFloat(69+40, y, value, 8, 4);   // 用当前前景色显示数值
        y += 32;
        item = item->next;
        r++;
    }while(item != page->child);
}


void Attitude_Refresh(void){
    MenuNode *item = page->child;
    uint16_t y = 48;
    uint8_t r = 0;
    do{
        if(r >= 3) break;                   //只画三行数据

        if(item == selected){                // 当前选中行：保持蓝底白字
            NEUQ_LCD200_SetPenColor(LCD_WHITE);
            NEUQ_LCD200_SetBackColor(LCD_BLUE);
        }else{                               // 其他行：白底黑字
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
        }

        NEUQ_LCD200_ClearRect(104+40, y, 220, 24);          // 按当前背景色清数值区
        float value = ((item == selected) && (editing)) ? edit_value : (*item->info->value);
        NEUQ_LCD200_DisplayFloat(104+40, y, value, 8, 4);   // 用当前前景色显示数值
        y += 32;
        item = item->next;
        r++;

    }while(item != page->child);
}

// Flash上电读取
void Config_Load(void){
    Config_Flash_t cfg;

    // 兜底：先无条件回到规定默认值
    Kalman_q_cnt = KALMAN_DEFAULT_Q;
    Kalman_r_cnt = KALMAN_DEFAULT_R;

    if(NEUQ_Flash_read_sector(NEUQ_FLASH_LAST_SECTOR_ID,(uint8_t*)&cfg, sizeof(cfg)) != NEUQ_FLASH_OK){
        return;                              // 读失败：保持默认值
    }

    uint8_t valid = 1;                       // 整条记录是否有效

    if(cfg.select_axis >= 0.0f && cfg.select_axis <= 5.0f &&
       (cfg.transmit_switch == 0.0f || cfg.transmit_switch == 1.0f)){
        Select_transmission_cnt = cfg.select_axis;
        Transmission_switch_cnt = cfg.transmit_switch;
    }else{
        valid = 0;
    }

    // 滤波参数：严格要求 >0 且在合理范围（0和NaN都会被拒绝）
    if(cfg.kalman_q >= 0.001f && cfg.kalman_q <= 10.0f &&
       cfg.kalman_r >= 0.01f  && cfg.kalman_r <= 5000.0f){
        Kalman_q_cnt = cfg.kalman_q;
        Kalman_r_cnt = cfg.kalman_r;
    }else{
        valid = 0;
    }

    // 数据非法：用规定默认值写回Flash，下次上电就有有效记录
    if(valid == 0){
        cfg.select_axis     = Select_transmission_cnt;
        cfg.transmit_switch = Transmission_switch_cnt;
        cfg.kalman_q        = Kalman_q_cnt;
        cfg.kalman_r        = Kalman_r_cnt;
        NEUQ_Flash_write_sector(NEUQ_FLASH_LAST_SECTOR_ID,(uint8_t*)&cfg, sizeof(cfg));
    }
}

// 保存写入Flash第511扇区
void Config_Save(void){
    Config_Flash_t cfg;
    cfg.select_axis     = Select_transmission_cnt;
    cfg.transmit_switch = Transmission_switch_cnt;
    cfg.kalman_q        = Kalman_q_cnt;
    cfg.kalman_r        = Kalman_r_cnt;
    NEUQ_Flash_write_sector(NEUQ_FLASH_LAST_SECTOR_ID,(uint8_t*)&cfg, sizeof(cfg));
}
