#include "NEUQ_lib_include.h"

NEUQ_Menu_Params params;
typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0,        // 一级：主菜单
    MENU_AXITS,           // 一级：六轴数据显示
    MENU_WAVEFORM,        // 一级：波形与滤波设置
    MENU_ATTITUDE,        // 一级：姿态解算


    MENU_SEL_AXIS,        // 选择发送轴
    MENU_SEND_EN,         // 发送开关
    MENU_ACC_FILTER_K,    // 加速度滤波系数
    MENU_GYRO_FILTER_K,   // 角速度滤波系数
    MENU_SAVE_FLASH,      // 保存到Flash
    MENU_GYRO_CAL,        // 陀螺零偏重标定

    MENU_COUNT,
} MenuID;

typedef struct{
    const char*name;
    MenuID parent_id;
    int32_t *value;
    int32_t min;
    int32_t max;
    int32_t step;
    void (*show_page)(void);
}MenuConfig;

static void Axits_Show(void);
static void Attitude_Show(void);
static void Config_Load(void);
static void Config_Save(void);

static const MenuConfig MenuTable[MENU_COUNT] = {
    [MENU_MAIN]      = {"MAIN menu",   MENU_NONE,       NULL,                0, 0, 0,   NULL},
    [MENU_AXITS]     = {"AXITS",       MENU_MAIN,       NULL,                0, 0, 0,   Axits_Show},
    [MENU_WAVEFORM]  = {"WAVEFORM",    MENU_MAIN,       NULL,                0, 0, 0,   NULL},
    [MENU_ATTITUDE]  = {"ATTITUDE",    MENU_MAIN,       NULL,                0, 0, 0,   Attitude_Show},

    [MENU_SEL_AXIS]  = {"SEL AXIS",    MENU_WAVEFORM,   &params.send_axis,   0, 5, 1,   NULL},
    [MENU_SEND_EN]   = {"SEND EN",     MENU_WAVEFORM,   &params.send_enable, 0, 1, 1,   NULL},
    [MENU_ACC_FILTER_K]  = {"ACC FILT K",  MENU_WAVEFORM,   &params.acc_filter_k,  0, 100, 5, NULL},
    [MENU_GYRO_FILTER_K] = {"GYRO FILT K", MENU_WAVEFORM,   &params.gyro_filter_k, 0, 100, 5, NULL},
    [MENU_SAVE_FLASH]= {"SAVE FLASH",  MENU_WAVEFORM,   &params.save_flag,   0, 1, 1,   NULL},
    [MENU_GYRO_CAL]  = {"GYRO CAL",    MENU_WAVEFORM,   &params.gyro_cal,    0, 1, 1,   NULL},
};


typedef struct MenuNode{
  const MenuConfig *info;
  struct MenuNode *parent;
  struct MenuNode *child;
  struct MenuNode *prev;
  struct MenuNode *next;
}MenuNode;

static MenuNode nodes[MENU_COUNT];
static MenuNode *page;
static MenuNode *selected;
static uint8_t editing;

static  void Menu_BuildLinks(void){
    for(uint8_t i=0;i<MENU_COUNT;i++){
      nodes[i].info=&MenuTable[i];
      nodes[i].parent=NULL;
      nodes[i].child=NULL;
      nodes[i].prev=NULL;
      nodes[i].next=NULL;
    }
      for(uint8_t i=0;i<MENU_COUNT;i++){
        MenuNode *node =&nodes[i];
        MenuNode *parent;
        if(node->info->parent_id==MENU_NONE)continue;
       parent=&nodes[node->info->parent_id];
       node->parent=parent;
        MenuNode *first = parent->child;

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
static int32_t edit_value=0;


#define CONFIG_MAGIC   0x4D505530UL
#define CONFIG_SECTOR   510

typedef struct{ 
    uint32_t magic;
    int32_t  send_axis;
    int32_t  send_enable;
    int32_t  acc_filter_k;
    int32_t  gyro_filter_k;
}ConfigBlock;

static void Config_Load(void){
    ConfigBlock cfg;
    if(NEUQ_Flash_read_sector(CONFIG_SECTOR,(uint8_t*)&cfg,sizeof(cfg))!=NEUQ_FLASH_OK)return;
    if(cfg.magic!=CONFIG_MAGIC)return;
    params.send_axis=cfg.send_axis;
    params.send_enable=cfg.send_enable;
    params.acc_filter_k=cfg.acc_filter_k;
    params.gyro_filter_k=cfg.gyro_filter_k;
}

static void Config_Save(void){
    ConfigBlock cfg;
    cfg.magic=CONFIG_MAGIC;
    cfg.send_axis=params.send_axis;
    cfg.send_enable=params.send_enable;
    cfg.acc_filter_k=params.acc_filter_k;
    cfg.gyro_filter_k=params.gyro_filter_k;
    NEUQ_Flash_write_sector(CONFIG_SECTOR,(uint8_t*)&cfg,sizeof(cfg));
}

//跳转逻辑
static uint8_t Menu_Update(KEY_EVENT_ENUM key){
    if(editing==1){
        const MenuConfig *item=selected->info;

        switch (key)
        {
        case KEY_UP_SHORT:
        case KEY_UP_LONG:
         edit_value+=item->step;
            break;
        case KEY_DOWN_SHORT:
        case KEY_DOWN_LONG:
         edit_value-=item->step;
            break;
        case KEY_FORWARD_SHORT:
        *item->value=edit_value;
        editing=0;
        if(selected->info==&MenuTable[MENU_GYRO_CAL] && edit_value==1){
            NEUQ_Attitude_Calibrate();   // 现场重标定零偏并保存（标定期间保持静止）
            *item->value=0;
        }
        if(selected->info==&MenuTable[MENU_SAVE_FLASH] && edit_value==1){
            Config_Save();     
            *item->value=0;     
        }
        break;
        case KEY_BACK_SHORT:
        editing=0;
        break;
        default:
        return 0;
        }
        if(edit_value < item->min) edit_value=item->min;
        if(edit_value > item->max) edit_value=item->max;
        return 1;
    }


    if(page->info->show_page!=NULL){
        if(key==KEY_BACK_SHORT){
            selected=page;
            page=page->parent;
            return 1;
        }
        return 0;
    }

    switch (key){
    case KEY_UP_SHORT:
    selected=selected->prev;
    break;
    case KEY_DOWN_SHORT:
    selected=selected->next;
    break;
    case KEY_FORWARD_SHORT:
    if(selected->child!=NULL){
     page=selected;
    selected=selected->child;
}else if(selected->info->value!=NULL){
    editing=1;
    edit_value=*selected->info->value;
}else if(selected->info->show_page!=NULL){
    page=selected;
}else{
    return 0;
}
    break;
    case KEY_BACK_SHORT:
    if(page->parent== NULL)return 0;
    selected=page;
    page=page->parent;
    break;
    default:{
        return 0;
    }
}
  return 1;
}

static char *axis_name[6]={"AX","AY","AZ","GX","GY","GZ"};

static void Axits_Show(void){
    uint16_t y=40;
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(8,8,"AXITS");

    for(uint8_t i=0;i<6;i++){
        NEUQ_LCD200_DisplayString(8,y,axis_name[i]);
        NEUQ_LCD200_DisplayFloat(40,y,sensor_raw[i],8,2);
        NEUQ_LCD200_DisplayFloat(120,y,sensor_filt[i],8,2);
        y+=28;
    }
}

static void Attitude_Show(void){
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(8,8,"ATTITUDE");
    NEUQ_LCD200_DisplayString(8,48,"roll");
    NEUQ_LCD200_DisplayFloat(80,48,attitude_roll,8,2);
    NEUQ_LCD200_DisplayString(8,80,"pitch");
    NEUQ_LCD200_DisplayFloat(80,80,attitude_pitch,8,2);
    NEUQ_LCD200_DisplayString(8,112,"yaw");
    NEUQ_LCD200_DisplayFloat(80,112,attitude_yaw,8,2);
}

static void Menu_Show(void){
    if(page->info->show_page!=NULL){
        page->info->show_page();
        return;
    }
    MenuNode *item=page->child;

    NEUQ_LCD200_Clear();
    char line[32];
    uint16_t y=48;
    snprintf(line,sizeof(line),"%s",page->info->name);
    NEUQ_LCD200_DisplayString(8,8,line);

    if(item!=NULL){
        do{
            char mark=' ';
            const MenuConfig *info=item->info;

            if(item==selected){
                mark=editing?'*':'>';
                 NEUQ_LCD200_SetPenColor(LCD_WHITE);
                 NEUQ_LCD200_SetBackColor(LCD_BLUE);
                 NEUQ_LCD200_ClearRect(8, y, 224,24);
            }
            if(info->value !=NULL){
               int32_t value =((item==selected)&&(editing)?edit_value:(*info->value));
               snprintf(line,sizeof(line),"%c %s:%ld",mark,info->name,value);
            }else{
               snprintf(line,sizeof(line),"%c %s",mark,info->name);
            }
            NEUQ_LCD200_DisplayString(8,y,line);

            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y+=32;
            item=item->next;

        }while(item!=page->child);
    }

}
void NEUQ_Menu_Init(void){
    params.send_axis=0;
    params.send_enable=0;
    params.acc_filter_k=20;
    params.gyro_filter_k=20;

    params.save_flag=0;
    params.gyro_cal=0;

    Config_Load();

    Menu_BuildLinks();
    page=&nodes[MENU_MAIN];
    selected=page->child;
    editing=0;

    NEUQ_LCD200_init();
     NEUQ_LCD200_SetPenColor(LCD_BLACK);
     NEUQ_LCD200_SetBackColor(LCD_WHITE);
     NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    Menu_Show();
}
   void NEUQ_Menu_Task(void){
    static uint32_t last_refresh=0;
    KEY_EVENT_ENUM key;
    key=NEUQ_Key_EventGet();
    if (Menu_Update(key)){
        Menu_Show();
        last_refresh=NEUQ_Get_Timer_Value_ms();
        return;
    }
    if(page->info->show_page!=NULL){
        uint32_t now=NEUQ_Get_Timer_Value_ms();
        if(now-last_refresh>=50){
            last_refresh=now;
            Menu_Show();
        }
    }
   }