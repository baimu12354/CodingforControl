#include "NEUQ_lib_include.h"
#include "gyr_data.h"
#include "wave.h"
#include "attitude.h"
//===================================

typedef struct{
    int32_t speed;
    int32_t accel;
    int32_t servo_center;
    int32_t servo_limit;
    int32_t filter_axis;           //0-5当前的轴
    int32_t filter_enable;         //0关 1开
    int32_t filter_param;          //滤波参数
}NEUQ_MENU_PARAM;
NEUQ_MENU_PARAM params;
//=====================k=====================
// static uint32_t battery_mv=0;
static uint8_t led_highlight=0;
static uint8_t filter_highlight= 0;
#define MENU_refresh 100
#define ADC_CH ADC1_CH3_PA3 //PA3模拟输入
#define ADC_RES ADC_12BIT  //12分辨率
#define ADC_VREF_MV   3300
#define ADC_FULL_SCALE  4096
#define BAT_DIV_RATIO  4 //电池电压=引脚电压*4
#define ADC_FILTER_CNT 8//均值滤波参数
//菜单 ID
typedef enum{
    MENU_NONE=-1,
    MENU_MAIN=0,
    MENU_LED,
    MENU_GYR_DATA,
    MENU_FILTER_SET,
    MENU_ATTITUDE,
    MENU_FILTER_CS,
    MENU_FILTER_SWITCH,
    MENU_FILTER_PARAM,
    MENU_COUNT
}MENU_ID;
typedef struct{
    const char*name;
    MENU_ID parent_ID;
    int32_t* value;
    int32_t min,max,step;
    uint8_t (*key_callback)(KEY_EVENT_ENUM key);
    void (*show_callback)(void);
}MenuConfig;



//=================================declaration=============================================
static uint8_t Led_Menu_Key_Handler(KEY_EVENT_ENUM key);
static void    Led_Menu_Show(void);
static uint8_t GYR_Menu_Key_Handler(KEY_EVENT_ENUM Key);
static void    GYR_Menu_Show(void);
static uint8_t Filter_Menu_Key_Handler(KEY_EVENT_ENUM key);
static void    Filter_Menu_Show(void);
static uint8_t Attitude_Menu_Key_Handler(KEY_EVENT_ENUM key);
static void    Attitude_Menu_Show(void);


//====================================菜单树构建==========================================
static const MenuConfig menutable[MENU_COUNT]={
    [MENU_MAIN]          ={"Main Menu"        ,MENU_NONE       ,NULL,0,0,0,                NULL,NULL},
    [MENU_LED]           ={"LED"          ,MENU_MAIN       ,NULL,0,0,0,                Led_Menu_Key_Handler,Led_Menu_Show},
    [MENU_GYR_DATA]      ={"6-Axis"     ,MENU_MAIN        ,NULL,0,0,0,               GYR_Menu_Key_Handler,GYR_Menu_Show},
    [MENU_FILTER_SET]    ={"Filter"   ,MENU_MAIN        ,NULL,0,0,0,              Filter_Menu_Key_Handler,Filter_Menu_Show},
    [MENU_ATTITUDE]      ={"Attitude"         ,MENU_MAIN        ,NULL,0,0,0,              Attitude_Menu_Key_Handler,Attitude_Menu_Show},
    [MENU_FILTER_CS]     ={"Axis"       ,MENU_FILTER_SET  ,&params.filter_axis,0,5,1,         NULL,NULL},
    [MENU_FILTER_SWITCH] ={"Send"         ,MENU_FILTER_SET  ,&params.filter_enable,0,1,1,       NULL,NULL},
    [MENU_FILTER_PARAM]  ={"Param"         ,MENU_FILTER_SET  ,&params.filter_param,1,100,1,        NULL,NULL},
};
typedef struct Menunode{
    const MenuConfig *info;
    struct Menunode *parent;
    struct Menunode *child;
    struct Menunode *prev;
    struct Menunode *next;
}Menunode;

static Menunode  nodes[MENU_COUNT];
static Menunode    *page=NULL;
static Menunode    *selected=NULL;
static uint8_t     editing=0;
static int32_t     edit_value=0;

//构建链表
static void menu_buildlinks(void){
    for(uint8_t i=0;i<MENU_COUNT;i++){
        nodes[i].info  =   &menutable[i];
        nodes[i].parent=   NULL;
        nodes[i].child=    NULL;
        nodes[i].prev=     NULL;
        nodes[i].next=     NULL;
    }
    for (uint8_t i = 0; i < MENU_COUNT; i++){
        Menunode *node=&nodes[i];
        if(node->info->parent_ID==MENU_NONE)continue;
        Menunode*parent=&nodes[node->info->parent_ID];
        node->parent=parent;
        //构建链表
        if(parent->child==NULL){
            parent->child=node;
            node->prev=node;
            node->next=node;
        }else{
            Menunode *first=parent->child;
            Menunode *last =first->prev;
            node->prev=last;
            last->next=node;
            node->next=first;
            first->prev=node;
        }
    }
}
//电压采集
// static void V_Updata(void){
//     uint16_t raw=adc_mean_filter_convert(ADC_CH,ADC_FILTER_CNT);
//     battery_mv=(uint32_t)raw*ADC_VREF_MV/ADC_FULL_SCALE*BAT_DIV_RATIO;
// }
//led显示===================================================================================================
static uint8_t Led_Menu_Key_Handler(KEY_EVENT_ENUM key){
    switch(key){
        case KEY_UP_SHORT:
        case KEY_UP_LONG:
        led_highlight=(led_highlight==0)?1:0;
        return 1;
        case KEY_DOWN_LONG:
        case KEY_DOWN_SHORT:
        led_highlight=(led_highlight==1)?0:1;
        return 1;
        case KEY_FORWARD_SHORT:
        case KEY_FORWARD_LONG:
        if(led_highlight==1){
        LED_Scheme_SetPending((uint8_t)((LED_Scheme_GetPending()+1)&3));
        return 1;}
        return 0;
        default:
            return 0;
    }
}
static void Led_Menu_Show(void){
    char line[32];
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(8,8,(char*)page->info->name);

    for(uint8_t i=0;i<2;i++){
        char mark =(i==led_highlight)?'>':' ';

        if(i==0)
        snprintf(line,sizeof(line),"%c NOW:%d",mark,(int)LED_Scheme_GetCurrent()+1);
        else if(i==1)
        snprintf(line,sizeof(line),"%c NEXT:%d",mark,(int)LED_Scheme_GetPending()+1);
        // else if(i==2)
        // snprintf(line,sizeof(line),"%c BAT:%lu.%02lu V",mark,
        //         (unsigned long)(battery_mv/1000),(unsigned long)((battery_mv%1000)/10));
        if(i==led_highlight){
        NEUQ_LCD200_SetPenColor(LCD_RED);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        }else{
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        }
        NEUQ_LCD200_ClearRect(0,48+i*32,224,24);
        NEUQ_LCD200_DisplayString(8,48+i*32,line);
    }
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_DisplayString(8,160,"FROWARD: Switch  BACK: Return");
}


//六轴数据显示================================================================================================
static uint8_t GYR_Menu_Key_Handler(KEY_EVENT_ENUM key){
    return 0;
}
static void GYR_Menu_Show(void){
    char line[32];
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(8,8,"6-Axis (MPU6050)");
    //获得加速度和角速度的函数

    //显示六行数据
    snprintf(line, sizeof(line), "AccX: %.2f ", acc_x_g); NEUQ_LCD200_DisplayString(8, 48, line);
    snprintf(line, sizeof(line), "AccY: %.2f ", acc_y_g); NEUQ_LCD200_DisplayString(8, 80, line);
    snprintf(line, sizeof(line), "AccZ: %.2f  ", acc_z_g); NEUQ_LCD200_DisplayString(8, 112, line);
    snprintf(line, sizeof(line), "GyroX: %.1f  ", gyro_x_dps); NEUQ_LCD200_DisplayString(8, 144, line);
    snprintf(line, sizeof(line), "GyroY: %.1f   ", gyro_y_dps); NEUQ_LCD200_DisplayString(8, 176, line);
    snprintf(line, sizeof(line), "GyroZ: %.1f  ", gyro_z_dps); NEUQ_LCD200_DisplayString(8, 208, line);
    NEUQ_LCD200_DisplayString(112,48, "m/s^2");
    NEUQ_LCD200_DisplayString(112,80, "m/s^2");
    NEUQ_LCD200_DisplayString(112,112, "m/s^2");
    NEUQ_LCD200_DisplayString(112,144, "deg/s");
    NEUQ_LCD200_DisplayString(112,176, "deg/s");
    NEUQ_LCD200_DisplayString(112,208, "deg/s");
    NEUQ_LCD200_DisplayString(8, 240, "BACK: Return");
}


static uint8_t Filter_Menu_Key_Handler(KEY_EVENT_ENUM key){
    return 0;
}
//==============================filter显示==========================================
static void Filter_Menu_Show(void){
    char line[32];
    const char *axis_name[6] = {"AX", "AY", "AZ", "GX", "GY", "GZ"};
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(8,8,(char*)page->info->name);
    int32_t ax = params.filter_axis;
    int32_t enable = params.filter_enable;
    int32_t param  = params.filter_param;
    if (ax < 0 || ax > 5) ax = 0;

    if      (selected == &nodes[MENU_FILTER_CS])     {filter_highlight = 0;if(editing)ax=edit_value;}
    else if (selected == &nodes[MENU_FILTER_SWITCH]) {filter_highlight = 1;if(editing)enable=edit_value;}
    else if (selected == &nodes[MENU_FILTER_PARAM])  {filter_highlight = 2;if(editing)param=edit_value;}

    for(uint8_t i=0;i<3;i++){
        char mark =(i==filter_highlight)?(editing ? '*' : '>'):' ';
        if(i==0)
        snprintf(line,sizeof(line),"%c Axis: %s",mark,axis_name[ax]);
        else if(i==1)
        snprintf(line,sizeof(line),"%c Send: %s",mark,enable ? "ON " : "OFF");
        else if(i==2)
         snprintf(line, sizeof(line), "%c Param: %ld",mark,(long)param);

        if(i==filter_highlight){
        NEUQ_LCD200_SetPenColor(LCD_RED);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        }else{
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        }
        NEUQ_LCD200_ClearRect(0,48+i*32,224,24);
        NEUQ_LCD200_DisplayString(8,48+i*32,line);
    }
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
        if (editing) {
        NEUQ_LCD200_DisplayString(8, 160, "FORWARD: Confirm  BACK: Cancel");
    }   else {
        NEUQ_LCD200_DisplayString(8, 160, "FORWARD: Enter  BACK: Return");
    }
}



//====================================================Attitude处理==============================================
static uint8_t Attitude_Menu_Key_Handler(KEY_EVENT_ENUM key){
    return 0;
}

static void Attitude_Menu_Show(void){
NEUQ_LCD200_Clear();
NEUQ_LCD200_DisplayString(8,8,"Pose calculating");
//数据
    NEUQ_LCD200_DisplayString(8,   48, "Pitch:");
    NEUQ_LCD200_DisplayFloat (80,  48,  g_attitude.pitch,6,2);
    NEUQ_LCD200_DisplayString(140, 48, "deg");

    NEUQ_LCD200_DisplayString(8,   80, "Roll:");
    NEUQ_LCD200_DisplayFloat (80,  80,  g_attitude.roll,6,2);
    NEUQ_LCD200_DisplayString(140, 80, "deg");

    NEUQ_LCD200_DisplayString(8,   112, "Yaw:");
    NEUQ_LCD200_DisplayFloat (80,  112,  g_attitude.yaw,6,2);
    NEUQ_LCD200_DisplayString(140, 112, "deg");

    NEUQ_LCD200_DisplayString(8,   160, "BACK: Return");}
//=========================================按键处理============================================
static uint8_t menu_update(KEY_EVENT_ENUM key){
    if(page->info->key_callback!=NULL){
        if(page->info->key_callback(key)){
            return 1;
        }
    }
     if(selected == NULL){
        if(key == KEY_BACK_SHORT || key == KEY_BACK_LONG){
            if(page->parent == NULL) return 0;
            selected = page;
            page = page->parent;
            NEUQ_LCD200_Clear();
            return 1;
        }
        return 0;   // 其他按键忽略
    }


    if(editing ==1){
        const MenuConfig*item=selected->info;
        switch(key){
            case KEY_UP_SHORT:
            case KEY_UP_LONG:
            edit_value+=item->step;
            if(edit_value>item->max)edit_value=item->min;
            break;
            case KEY_DOWN_LONG:
            case KEY_DOWN_SHORT:
            edit_value-=item->step;
            if(edit_value<item->min)edit_value=item->max;
            break;
            case KEY_FORWARD_SHORT:
            case KEY_FORWARD_LONG:
            if (item->value!=NULL){
            *item->value=edit_value;
            g_wave_save_req=1;
            }
            editing=0;
            break;
            case KEY_BACK_LONG:
            case KEY_BACK_SHORT:
            editing=0;
            break;
            default:return 0;
        }
        return 1;
    }
    switch (key)
    {
    case KEY_UP_SHORT:
    case KEY_UP_LONG:
    selected=selected->prev;
    break;
    case KEY_DOWN_SHORT:
    case KEY_DOWN_LONG:
    selected=selected->next;
    break;
    case KEY_FORWARD_SHORT:
    case KEY_FORWARD_LONG:
    if(selected->child!=NULL){
        page=selected;
        selected=selected->child;
    }else if (selected->info->value!=NULL)
    {
    editing=1;
    edit_value=*selected->info->value;
    }else if(selected->info->show_callback!=NULL){
        page=selected;
        selected=NULL;
        NEUQ_LCD200_Clear();
    }
    break;
    case KEY_BACK_SHORT:
    case KEY_BACK_LONG:
    if(page->parent==NULL)return 0;
    selected=page;
    page=page->parent;
    NEUQ_LCD200_Clear();
    break;
    default :return 0;
}return 1;}

// ==================== 显示逻辑 ====================
static uint8_t Menu_show(void){
    if(page==NULL)return 0;
    if(page->info->show_callback!=NULL){
        page->info->show_callback();
        return 1;
    }
    Menunode *item=page->child;

    // NEUQ_LCD200_Clear();
    char line[32];
    uint16_t y=48;
    snprintf (line,sizeof(line),"%s",page->info->name);
    NEUQ_LCD200_DisplayString(8,8,line);
    if(item !=NULL){
        do{
            char mark=' ';
            const MenuConfig*info=item->info;
            if(item ==selected){
                mark=editing?'*':'>';
                NEUQ_LCD200_SetPenColor(LCD_RED);
                NEUQ_LCD200_SetBackColor(LCD_WHITE);
                NEUQ_LCD200_ClearRect(8,y,224,24);
            }
            if(info->value!=NULL){
                int32_t value =((item==selected)&&(editing))?edit_value:(*info->value);
                snprintf(line,sizeof(line),"%c %s:%ld",mark,info->name,value);
            }else{
                snprintf(line,sizeof(line),"%c %s",mark,info->name);
            }NEUQ_LCD200_DisplayString(8,y,line);
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y+=32;
            item=item->next;
        }while (item!=page->child);
    }
    return 1;
}
 //=================初始化============================
void NEUQ_Menu_Init(void){
   
    params.speed=0;
    params.accel=0;
    params.servo_center=0;
    params.servo_limit=0;
    params.filter_axis=0;
    params.filter_enable=0;
    params.filter_param=20;
    led_highlight=0;
    filter_highlight= 0;
    menu_buildlinks();
    // 注意：LCD 已在 User_Main() 里初始化过，这里不要再重复初始化，
    // 否则会再次拉低背光并重发复位序列（屏幕会闪一下，甚至卡住变暗）
    page=&nodes[MENU_MAIN];
    selected=page->child;
    editing=0;
    // 覆盖库默认的 Font24（12 宽字库与显示函数不匹配会乱码），改用 8 宽 Font16
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    // V_Updata();
    Wave_Init();
    Menu_show();
    // NEUQ_LCD200_SetPenColor(LCD_RED);
// NEUQ_LCD200_SetBackColor(LCD_WHITE);
// NEUQ_LCD200_Clear();                          // 先刷白
// NEUQ_LCD200_DisplayString(10, 10, "ABC123");  // 再写红字
// while(1);                                     // 停在这里看
}
void NEUQ_Menu_Task(void){
    static uint32_t last_tick=0;
    //实时处理按键
    KEY_EVENT_ENUM key=NEUQ_KEY_EventGet();
    if(menu_update(key)){
        Menu_show();
    }
    if(g_wave_save_req){
        g_wave_save_req=0;
        Wave_FlashSave();
    }
    uint32_t now=LED_GetTick10ms();
    if((now-last_tick)>=MENU_refresh){
        last_tick=now;
        // V_Updata();
        //mpu650采集函数
        Menu_show();
    }
}


