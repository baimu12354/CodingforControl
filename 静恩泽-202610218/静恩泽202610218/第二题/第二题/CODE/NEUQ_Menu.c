#include "NEUQ_lib_include.h"
#include "NEUQ_Imu.h"

// 页面编号
typedef enum{
    MENU_NONE=-1,
    MENU_MAIN=0,      // 一级菜单：三个入口
    MENU_IMU_DATA,    // 二级：六轴数据显示
    MENU_WAVE,        // 二级：波形与滤波设置
    MENU_ATTITUDE,    // 二级：姿态解算
    MENU_WAVE_AXIS,   // 波形设置子项：选择发送轴
    MENU_WAVE_SEND,   // 波形设置子项：开启/停止发送
    MENU_WAVE_FILTER, // 波形设置子项：滤波系数
    MENU_WAVE_SAVE,   // 波形设置子项：保存到Flash
    MENU_COUNT
}MenuID;

// 菜单项类型
typedef enum{
    MENU_TYPE_LIST=0,   // 列表容器
    MENU_TYPE_DATA,     // 动态数据页
    MENU_TYPE_PARAM,    // 可编辑整数参数
    MENU_TYPE_ENUM,     // 枚举选项
    MENU_TYPE_TOGGLE,   // 开关
    MENU_TYPE_ACTION,   // 动作
}MenuItemType;

// 页面结构
typedef struct{
    const char *name;       // 页面名称
    MenuID parent_id;       // 父页面
    MenuItemType type;      // 页面类型
    int32_t *value;         // 参数指针
    int32_t min;
    int32_t max;
    int32_t step;
}MenuConfig;

static const char *onoff_names[2]={"OFF","ON"};

static const MenuConfig MenuTable[MENU_COUNT]={
    [MENU_MAIN]       ={"Main Menu",  MENU_NONE,  MENU_TYPE_LIST,   NULL,0,0,0},
    [MENU_IMU_DATA]   ={"IMU Data",   MENU_MAIN,  MENU_TYPE_DATA,   NULL,0,0,0},
    [MENU_WAVE]       ={"Wave&Filter",MENU_MAIN,  MENU_TYPE_LIST,   NULL,0,0,0},
    [MENU_ATTITUDE]   ={"Attitude",   MENU_MAIN,  MENU_TYPE_DATA,   NULL,0,0,0},
    [MENU_WAVE_AXIS]  ={"Axis",   MENU_WAVE, MENU_TYPE_ENUM,   &imu_cfg.send_axis,   0,IMU_AXIS_COUNT-1,1},
    [MENU_WAVE_SEND]  ={"Send",   MENU_WAVE, MENU_TYPE_TOGGLE, &imu_cfg.send_enable, 0,1,1},
    [MENU_WAVE_FILTER]={"Filter", MENU_WAVE, MENU_TYPE_PARAM,  &imu_cfg.filter_k,    0,900,10},
    [MENU_WAVE_SAVE]  ={"Save",   MENU_WAVE, MENU_TYPE_ACTION, NULL,0,0,0},
};

// 菜单树节点
typedef struct MenuNode{
    const MenuConfig *info;
    struct MenuNode *parent;
    struct MenuNode *child;
    struct MenuNode *next;
    struct MenuNode *prev;
}MenuNode;

static MenuNode nodes[MENU_COUNT];
static MenuNode *page;      // 当前页面
static MenuNode *selected;  // 当前选中项
static uint8_t editing=0;
static int32_t edit_value=0;

// 建立父子/同级关系
static void Menu_BuildLinks(void){
    for(uint8_t i=0;i<MENU_COUNT;i++){
        nodes[i].info=&MenuTable[i];
        nodes[i].parent=NULL;
        nodes[i].child=NULL;
        nodes[i].next=NULL;
        nodes[i].prev=NULL;
    }
    for(uint8_t i=0;i<MENU_COUNT;i++){
        MenuNode *node=&nodes[i];
        if(node->info->parent_id==MENU_NONE) continue;
        MenuNode *parent=&nodes[node->info->parent_id];
        node->parent=parent;

        MenuNode *first=parent->child;
        if(first==NULL){
            parent->child=node;
            node->prev=node;
            node->next=node;
        }else{
            MenuNode *last=first->prev;
            node->prev=last;
            last->next=node;
            node->next=first;
            first->prev=node;
        }
    }
}

static MenuID Menu_Index(const MenuNode *node){
    return (MenuID)(node->info-MenuTable);
}

static void Menu_DoAction(const MenuNode *item){
    switch(Menu_Index(item)){
        case MENU_WAVE_SAVE:
            NEUQ_Imu_SaveConfig();
            break;
        default:
            break;
    }
}

// 把菜单项格式化为一行显示文本
static void Menu_FormatItem(const MenuNode *item,char *line,uint16_t len,char mark){
    const MenuConfig *info=item->info;
    switch(info->type){
        case MENU_TYPE_ENUM:
        case MENU_TYPE_TOGGLE:
        case MENU_TYPE_PARAM:{
            int32_t val=(item==selected && editing)?edit_value:*info->value;
            if(info->type==MENU_TYPE_ENUM){
                snprintf(line,len,"%c %s: %s",mark,info->name,NEUQ_Imu_AxisName(val));
            }else if(info->type==MENU_TYPE_TOGGLE){
                snprintf(line,len,"%c %s: %s",mark,info->name,onoff_names[val?1:0]);
            }else{
                snprintf(line,len,"%c %s: %ld",mark,info->name,(long)val);
            }
            break;
        }
        case MENU_TYPE_ACTION:
            snprintf(line,len,"%c %s [OK]",mark,info->name);
            break;
        default:  // LIST / DATA
            snprintf(line,len,"%c %s",mark,info->name);
            break;
    }
}

// 键处理：编辑模式优先，其次浏览模式
static uint8_t MenuUpdate(KEY_EVENT_ENUM key){
    // 编辑模式（PARAM / ENUM）
    if(editing){
        const MenuConfig *info=selected->info;
        switch(key){
            case KEY_UP_SHORT:
            case KEY_UP_LONG:
                edit_value+=info->step;
                break;
            case KEY_DOWN_SHORT:
            case KEY_DOWN_LONG:
                edit_value-=info->step;
                break;
            case KEY_FORWARD_SHORT:
                *info->value=edit_value;    // 确认并应用
                editing=0;
                break;
            case KEY_BACK_SHORT:
                editing=0;                  // 取消
                break;
            default:
                return 0;
        }
        if(edit_value>info->max) edit_value=info->max;
        if(edit_value<info->min) edit_value=info->min;
        return 1;
    }

    // 数据页：无选中项，仅返回键有效
    if(selected==NULL){
        if(key==KEY_BACK_SHORT && page->parent!=NULL){
            selected=page;
            page=page->parent;
            return 1;
        }
        return 0;
    }

    // 浏览模式
    const MenuConfig *info=selected->info;
    switch(key){
        case KEY_UP_SHORT:
            if(page->child!=NULL) selected=selected->prev;
            break;
        case KEY_DOWN_SHORT:
            if(page->child!=NULL) selected=selected->next;
            break;
        case KEY_FORWARD_SHORT:
            switch(info->type){
                case MENU_TYPE_LIST:
                case MENU_TYPE_DATA:
                    page=selected;
                    selected=selected->child;
                    break;
                case MENU_TYPE_PARAM:
                case MENU_TYPE_ENUM:
                    editing=1;
                    edit_value=*info->value;
                    break;
                case MENU_TYPE_TOGGLE:
                    *info->value=(*info->value)?0:1;
                    break;
                case MENU_TYPE_ACTION:
                    Menu_DoAction(selected);
                    break;
                default:
                    break;
            }
            break;
        case KEY_BACK_SHORT:
            if(page->parent!=NULL){
                selected=page;
                page=page->parent;
            }
            break;
        default:
            return 0;
    }
    return 1;
}

// 在指定行绘制“标签 数值 单位”，先清行避免残影
static void Menu_DrawValueRow(const char *label,float value,const char *unit,
                              uint8_t decimal,uint16_t y){
    NEUQ_LCD200_ClearRect(8,y,312,24);
    NEUQ_LCD200_DisplayString(8,y,(char *)label);
    NEUQ_LCD200_DisplayFloat(72,y,value,7,decimal);
    NEUQ_LCD200_DisplayString(248,y,(char *)unit);
}

static void Menu_DrawAngleRow(const char *label,float angle,uint16_t y){
    NEUQ_LCD200_ClearRect(8,y,312,24);
    NEUQ_LCD200_DisplayString(8,y,(char *)label);
    NEUQ_LCD200_DisplayFloat(96,y,angle,8,1);
    NEUQ_LCD200_DisplayString(8+24*14,y,(char *)"deg");
}

static void Menu_DrawImuData(void){
    // 三轴加速度 (g)
    Menu_DrawValueRow("Ax",NEUQ_Imu_Raw(IMU_AXIS_ACC_X), "g",  2,40);
    Menu_DrawValueRow("Ay",NEUQ_Imu_Raw(IMU_AXIS_ACC_Y), "g",  2,68);
    Menu_DrawValueRow("Az",NEUQ_Imu_Raw(IMU_AXIS_ACC_Z), "g",  2,96);
    // 三轴角速度 (dps)
    Menu_DrawValueRow("Gx",NEUQ_Imu_Raw(IMU_AXIS_GYRO_X),"dps",1,132);
    Menu_DrawValueRow("Gy",NEUQ_Imu_Raw(IMU_AXIS_GYRO_Y),"dps",1,160);
    Menu_DrawValueRow("Gz",NEUQ_Imu_Raw(IMU_AXIS_GYRO_Z),"dps",1,188);
}

static void Menu_DrawAttitude(void){
    Menu_DrawAngleRow("Roll", NEUQ_Imu_Roll(), 48);
    Menu_DrawAngleRow("Pitch",NEUQ_Imu_Pitch(),84);
    Menu_DrawAngleRow("Yaw",  NEUQ_Imu_Yaw(),  120);
    NEUQ_LCD200_ClearRect(8,160,312,48);
    NEUQ_LCD200_DisplayString(8,168,(char *)"Update: 100Hz");
    NEUQ_LCD200_DisplayString(8,192,(char *)"Yaw: relative");
}

static void Menu_DrawData(void){
    if(page==&nodes[MENU_IMU_DATA]){
        Menu_DrawImuData();
    }else if(page==&nodes[MENU_ATTITUDE]){
        Menu_DrawAttitude();
    }
}

static void Menu_Show(void){
    NEUQ_LCD200_Clear();

    char title[32];
    snprintf(title,sizeof(title),"%s",page->info->name);
    NEUQ_LCD200_DisplayString(8,8,title);

    if(page->info->type==MENU_TYPE_DATA){
        Menu_DrawData();
        return;
    }

    MenuNode *item=page->child;
    if(item!=NULL){
        uint16_t y=48;
        do{
            char line[32];
            char mark=' ';
            if(item==selected){
                mark=editing?'*':'>';
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8,y,312,24);
            }
            Menu_FormatItem(item,line,sizeof(line),mark);
            NEUQ_LCD200_DisplayString(8,y,line);
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y+=32;
            item=item->next;
        }while(item!=page->child);
    }
}

void NEUQ_Menu_Init(void){
    Menu_BuildLinks();
    page=&nodes[MENU_MAIN];
    selected=page->child;
    editing=0;
    edit_value=0;
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    Menu_Show();
}

void NEUQ_Menu_Task(void){
    KEY_EVENT_ENUM key=NEUQ_Key_EventGet();
    if(MenuUpdate(key)){
        Menu_Show();     // 按键引起跳转/编辑时整页重绘
    }else if(page!=NULL && page->info->type==MENU_TYPE_DATA){
        static uint32_t last_refresh=0;
        uint32_t now=HAL_GetTick();
        if((now-last_refresh)>=100){   // 数据页每 100ms 动态刷新一次
            last_refresh=now;
            Menu_DrawData();
        }
    }
}
