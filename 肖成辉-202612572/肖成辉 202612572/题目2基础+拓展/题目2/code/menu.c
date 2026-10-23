#include "menu.h"
#include "key.h"
Menu_Params param;


#define PARAM_FLASH_SECTOR  (510U)

#define PARAM_FLASH_MAGIC   (0x4E455551UL)


typedef struct {
    uint32_t magic;      // 数据有效标识
    int16_t  axis;       // param.axis
    int16_t  start;      // param.start
    int16_t  enable[6];  // datatable[0..5].enable
} Param_Storage_t;


static void Param_Save(void)
{
    Param_Storage_t storage;

    storage.magic = PARAM_FLASH_MAGIC;
    storage.axis  = param.axis;
    storage.start = param.start;
    for (uint8_t i = 0; i < 6; i++) {
        storage.enable[i] = datatable[i].enable;
    }
    NEUQ_Flash_write_sector(PARAM_FLASH_SECTOR, (const uint8_t *)&storage, sizeof(storage));
}


static void Param_Load(void)
{
    Param_Storage_t storage;

    if (NEUQ_Flash_read_sector(PARAM_FLASH_SECTOR, (uint8_t *)&storage, sizeof(storage)) == NEUQ_FLASH_OK &&
        storage.magic == PARAM_FLASH_MAGIC) {
        param.axis  = storage.axis;
        param.start = storage.start;
        for (uint8_t i = 0; i < 6; i++) {
            datatable[i].enable = storage.enable[i];
        }
    } else if (NEUQ_Flash_get_capacity() != 0U) {
          
        Param_Save();
    }
}

typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_MPU5060,
    MENU_VOFA,
    MENU_QAE,
    MENU_GX,
    MENU_GY,
    MENU_GZ,
    MENU_AX,
    MENU_AY,
    MENU_AZ,
    MENU_AXIS,
    MENU_FILTER_AX,
    MENU_FILTER_AY,
    MENU_FILTER_AZ,
    MENU_FILTER_GX,
    MENU_FILTER_GY,
    MENU_FILTER_GZ,
    MENU_VOFA_START,
    MENU_YAW,
    MENU_PITCH,
    MENU_ROLL,
   
    MENU_COUNT,
}MENUID;
typedef struct{
    const char *name;
    MENUID parent_id;
    int16_t *value;
    const float *float_value;
    int16_t min;
    int16_t max;
    int16_t step;
}MENU_CONFIG;

static const MENU_CONFIG MENU_Table[MENU_COUNT]=
{[MENU_MAIN] = {"Main menu",MENU_NONE,NULL,NULL,0,0,0},
 [MENU_MPU5060] = {"Mpu5060",MENU_MAIN,NULL,NULL,0,0,0},
 [MENU_GX] = {"Gx",MENU_MPU5060,NULL,&datatable[3].rawdata,0,0,0},
 [MENU_GY] = {"Gy",MENU_MPU5060,NULL,&datatable[4].rawdata,0,0,0},
 [MENU_GZ] = {"Gz",MENU_MPU5060,NULL,&datatable[5].rawdata,0,0,0},
 [MENU_AX] = {"Ax",MENU_MPU5060,NULL,&datatable[0].rawdata,0,0,0},
 [MENU_AY] = {"Ay",MENU_MPU5060,NULL,&datatable[1].rawdata,0,0,0},
 [MENU_AZ] = {"Az",MENU_MPU5060,NULL,&datatable[2].rawdata,0,0,0},
 [MENU_VOFA] = {"Vofa",MENU_MAIN,NULL,NULL,0,0,0},
 [MENU_AXIS] = {"Axis",MENU_VOFA,&param.axis,NULL,1,6,1},
 [MENU_FILTER_GX] = {"Filter_Gx",MENU_VOFA,&datatable[3].enable,NULL,0,1,1},
 [MENU_FILTER_GY] = {"Filter_Gy",MENU_VOFA,&datatable[4].enable,NULL,0,1,1},
 [MENU_FILTER_GZ] = {"Filter_Gz",MENU_VOFA,&datatable[5].enable,NULL,0,1,1},
 [MENU_FILTER_AX] = {"Filter_Ax",MENU_VOFA,&datatable[0].enable,NULL,0,1,1},
 [MENU_FILTER_AY] = {"Filter_Ay",MENU_VOFA,&datatable[1].enable,NULL,0,1,1},
 [MENU_FILTER_AZ] = {"Filter_Az",MENU_VOFA,&datatable[2].enable,NULL,0,1,1},
 [MENU_VOFA_START] = {"Start",MENU_VOFA,&param.start,NULL,0,1,1},
 [MENU_QAE] = {"Qae",MENU_MAIN,NULL,NULL,0,0,0},
 [MENU_YAW] = {"Yaw",MENU_QAE,NULL,&param.yaw,0,0,0},
 [MENU_PITCH] = {"Pitch",MENU_QAE,NULL,&param.pitch,0,0,0},
 [MENU_ROLL] = {"Roll",MENU_QAE,NULL,&param.roll,0,0,0},
 


};
typedef struct MenuNode
{
    const MENU_CONFIG *info;
    struct MenuNode *parent;
    struct MenuNode *child;
    struct MenuNode *prev;
    struct MenuNode *next;

}MenuNode;
static MenuNode nodes[MENU_COUNT];
static MenuNode *page;
static MenuNode *selected; 
static uint8_t editing;

static void MenuBuildLinks(void){
    for(uint8_t i = 0;i<MENU_COUNT;i++){
        nodes[i].info = &MENU_Table[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = NULL;
        nodes[i].next = NULL;
    }


    for(uint8_t i = 0;i<MENU_COUNT;i++){
        MenuNode *node =&nodes[i];
        MenuNode *parent;
        if(node->info->parent_id == MENU_NONE) continue;
        parent = &nodes[node->info->parent_id];
        node->parent = parent;

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
int32_t edit_value = 0;
static uint8_t MenuUpdate(KEY_EVENT_ENUM key){
    if(editing == 1){
        const MENU_CONFIG *item = selected->info;
        switch (key)
        {
        case KEY_UP_P:
            edit_value +=item->step;
            break;
        case KEY_DOWN_P:
            edit_value -=item->step;
            break;
        case KEY_FORWAED_P:
               *item->value = (int16_t)edit_value;
             editing = 0;
             Param_Save();
             break;
        case KEY_BACK_P:
             editing = 0;
             break;
        default:
            return 0;
        }
        if (edit_value<item->min)
        {
            edit_value = item->min;
        }
        if (edit_value>item->max)
        {
            edit_value = item->max;
        }
        return 1;
        
    }



    switch ((key))
    {
    case KEY_UP_P:
           selected = selected->prev;
            break;
        case KEY_DOWN_P:
            selected = selected->next;
            break;
        case KEY_FORWAED_P:
             if(selected->child != NULL){
                page = selected;
                selected = selected->child;
             }else if (selected->info->value != NULL)
             {editing = 1;
              edit_value = *selected->info->value;
             }else{return 0;}
             
             break;
        case KEY_BACK_P:
             if(page->parent == NULL){return 0;}
             selected = page;
             page = page->parent;
             break;
             default:
             return 0;
    }
    return 1;
}
static MenuNode *last_page = NULL;  

static void Menu_Show(void){

    MenuNode *item = page->child;
    char line[32];
    uint16_t y = 32;

    
    if (page != last_page) {
        last_page = page;
        NEUQ_LCD200_Clear();
        sprintf(line,"%s",page->info->name);
        NEUQ_LCD200_DisplayString(8,8,line);
    }
   
    

    if(item != NULL){
       do{ char mark = ' ';
        const MENU_CONFIG *info = item->info;
        int highlighted = (item == selected);
        NEUQ_LCD200_SetPenColor(highlighted ? LCD_WHITE : LCD_BLUE);
        NEUQ_LCD200_SetBackColor(highlighted ? LCD_BLUE : LCD_WHITE);
        NEUQ_LCD200_ClearRect(8,y,320,24);
        if (highlighted) {
            mark = editing ? '*' : '>';
        }
        if(info->float_value != NULL){
            sprintf(line,"%c %s:%.2f",mark,info->name,(double)*info->float_value);
        }else if(info->value != NULL){
            int32_t value = ((item == selected)&&(editing))? edit_value : (*info->value);
            sprintf(line,"%c %s:%ld",mark,info->name,(long)value);
        }else{
            sprintf(line,"%c %s",mark,info->name);
        }
        NEUQ_LCD200_DisplayString(8,y,line);
        NEUQ_LCD200_SetPenColor(LCD_BLUE);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        y+=24;
        item = item->next;
        }while(item != page->child);

    }
    
}

void MENU_Init(void){
    NEUQ_Flash_init();
    Param_Load();
    MenuBuildLinks();
    page = &nodes[MENU_MAIN];
    selected = page->child;
    editing = 0;
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLUE);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    Menu_Show();
}

void MENU_Task(void){
    KEY_EVENT_ENUM keyevent = KEYGet();
    if(MenuUpdate(keyevent)){
        Menu_Show();
    }
}

void MENU_Flash(void){
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    
    if (now - last_tick >= 50) {
        last_tick = now;
        Menu_Show();
    }
}