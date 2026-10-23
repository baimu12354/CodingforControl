#include "NEUQ_lib_include.h"

NEUQ_Menu_Params params = {0};
static const char * const WaveAxis[] = {"X Accel", "Y Accel", "Z Accel", "X Gyro", "Y Gyro" ,"Z Gyro", "None"}; 
uint16_t save_sector=NEUQ_FLASH_LAST_SECTOR_ID;
FilterSave fits;

//页面编号
typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_Rotate_Data,
    MENU_Rotate_Wave,
    MENU_Rotate_Status,
    ACCEL_X,
    ACCEL_Y,
    ACCEL_Z,
    GYRO_X,
    GYRO_Y,
    GYRO_Z,
    WAVE,
    FILTER_A,
    FILTER_G,
    ROLL,
    PITCH,
    YAW,
    MENU_COUNT
}MenuID;

//页面信息配置
typedef struct{
    const char *name;   //名称
    MenuID parent_id;   //父页面
    int32_t *value;     //参数，NULL为不编辑
    int32_t flt;        //判断是否为小数
    int32_t min;
    int32_t max;
    float step;
    const char * const *choices;
    int32_t choice_count;
}MenuConfig;

//菜单结构及参数
static const MenuConfig MenuTable[MENU_COUNT] = {
    [MENU_MAIN] = {"Main", MENU_NONE, NULL, 0, 0, 0, 0, NULL, 0},
    [MENU_Rotate_Data] = {"Rotate Data", MENU_MAIN, NULL, 0, 0, 0, 0, NULL, 0},
    [MENU_Rotate_Wave] = {"Waves", MENU_MAIN, NULL, 0, 0, 0, 0, NULL, 0},
    [MENU_Rotate_Status] = {"Pose", MENU_MAIN, NULL, 0, 0, 0, 0, NULL, 0},
    [ACCEL_X] = {"X Accel(g)     ", MENU_Rotate_Data, &params.ax, 1, -32768, 32768, 0, NULL, 0},
    [ACCEL_Y] = {"Y Accel(g)     ", MENU_Rotate_Data, &params.ay, 1, -32768, 32768, 0, NULL, 0},
    [ACCEL_Z] = {"Z Accel(g)     ", MENU_Rotate_Data, &params.az, 1, -32768, 32768, 0, NULL, 0},
    [GYRO_X] = {"X Ang(degree/s)", MENU_Rotate_Data, &params.gyro_x, 1, -32768, 32768, 0, NULL, 0},
    [GYRO_Y] = {"Y Ang(degree/s)", MENU_Rotate_Data, &params.gyro_y, 1, -32768, 32768, 0, NULL, 0},
    [GYRO_Z] = {"Z Ang(degree/s)", MENU_Rotate_Data, &params.gyro_z, 1, -32768, 32768, 0, NULL, 0},
    [FILTER_A] = {"Accel Filter", MENU_Rotate_Wave, &params.filt_a, 1, 10, 1000, 10, NULL, 0},
    [FILTER_G] = {"Gyro Filter", MENU_Rotate_Wave, &params.filt_g, 1, 10, 1000, 10, NULL, 0},
    [WAVE] = {"Sending Waves", MENU_Rotate_Wave, &params.wave_mode, 0, 0, 0, 0, WaveAxis, 7},
    [ROLL] = {"Roll   ", MENU_Rotate_Status, &params.roll, 1, -180000, 180000, 0, NULL, 0},
    [PITCH] = {"Pitch  ", MENU_Rotate_Status, &params.pitch, 1, -180000, 180000, 0, NULL, 0},
    [YAW] = {"Yaw    ", MENU_Rotate_Status, &params.yaw, 1, -180000, 180000, 0, NULL, 0},
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

        if (item->choices != NULL){
            switch(key){
                case KEY_UP_SHORT:
                case KEY_UP_LONG:  
                    edit_value++;            
                    break;
                case KEY_DOWN_SHORT:
                case KEY_DOWN_LONG: 
                    edit_value--;           
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
            }
            if (edit_value < 0)                   
                edit_value = item->choice_count - 1;
            if (edit_value >= item->choice_count) 
                edit_value = 0;
            return 1;
        }

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
            if (selected->info ==&MenuTable[FILTER_A] || selected->info == &MenuTable[FILTER_G]) {           
                fits.correct  = CORRECTION_NUM;
                fits.filtaccel = params.filt_a;
                fits.filtgyro = params.filt_g;
                NEUQ_Flash_write_sector(save_sector,(uint8_t *)&fits,sizeof(fits));
            }
                editing = 0;
                break;
            case KEY_ABORT_SHORT:
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
    const MenuConfig *info = page->info;

/*此处判断是否为六轴数据页，否正常清屏，是仅清屏数据那一块
  同时因为发现如果进入编辑模式，对应数据会卡住不动，所以刚
  好这里顺便把或六轴数据和姿态解算两页editing设为0，防止进
  入编辑模式*/
    if (page->info == &MenuTable[MENU_Rotate_Data]){
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        NEUQ_LCD200_ClearRect(220, 48, 100, 184);
        editing = 0;
    }else if (page->info == &MenuTable[MENU_Rotate_Status]){
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        NEUQ_LCD200_ClearRect(130, 48, 190, 184);
        editing = 0;
    }else{
        NEUQ_LCD200_Clear();
    }

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
                if (info->choices != NULL){   //枚举：显示文字
                    int32_t ids = ((item == selected) && (editing)) ? edit_value : (*info->value);
                    if (ids < 0) ids = 0;
                    if (ids >= info->choice_count) ids = info->choice_count - 1;
                    snprintf(line, sizeof(line), "%c %s:%s", mark, info->name, info->choices[ids]);

                } else if (info->flt == 0){
                    //此处判断是否为浮点数据，flt=0不是，=1是，main.c里浮点*1000，这里/1000
                    int32_t value = ((item == selected) && (editing)) ? edit_value : (*info->value);
                    snprintf (line,sizeof(line),"%c %s:%ld",mark,info->name,value);
                }else{
                        int32_t value = ((item == selected) && (editing)) ? edit_value : (*info->value);
                        char sign = ' ';
                        if (value < 0) {sign = '-';value = -value;}          // 负数的处理
                        int32_t ipart = value / 1000;
                        int32_t fpart = value % 1000;
                        snprintf (line, sizeof(line), "%c %s:%c%ld.%03ld", mark, info->name, sign, ipart, fpart);
                }
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
//    params.action = 1;
    Menu_BuildLinks();
    page = &nodes[MENU_MAIN];
    selected = page->child;
    editing = 0;

    //初始滤波数据
    params.filt_a = 150;   // 即 0.15
    params.filt_g = 90;   // 即 0.09

    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    Menu_Show();
}

void NEUQ_Menu_Task(void){
    KEY_EVENT_ENUM key;
    //计数器
    static uint8_t  throttle = 0;
    key = NEUQ_KEY_EventGet();
    if (Menu_Update(key)){
        Menu_Show();
    }
//防止刷新太快导致屏幕看着在闪（虽然还是有一点闪）
    throttle ++;
    if ((page == &nodes[MENU_Rotate_Data] || page == &nodes[MENU_Rotate_Status])&& throttle >= 15){
        Menu_Show();
        throttle = 0;
    }
}
