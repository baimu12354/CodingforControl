#include "NEUQ_lib_include.h"

NEUQ_Menu_Params params; //调用外部变量

#define WAVE_PARAMS_SECTOR 511  //宏定义扇区
#define WAVE_PARAMS_MAGIC 0x57415645  //宏定义魔数

typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_SIX_AXIS_DATA,
    MENU_WAVE_FILIER,
    MENU_ATTITUDE_SOLVER,
    MENU_WAVE_SELECT_AXIS,
    MENU_SEND_ENABLE,
    MENU_WAVE_ALPHA,
    MENU_COUNT
}MainMenu_Enum;  //枚举所有菜单页面及选项

WaveParams g_wave_params = {
    .magic = 0,
    .select_axis = 0,
    .send_enable = 0,
    .alpha_x100 = 20,
};   //全局变量定义，初始化

typedef struct{
    const char *name;
    MainMenu_Enum parent_id;
    int32_t *value;    //当前页面的参数地址
    int32_t min;
    int32_t max;
    int32_t step;
    PageType_Enum page_type;
}MenuConfig;  //定义结构体（页面名称，父页，参数地址，最大，最小，步长，页面类型）

static void Menu_SixAxis_Enter(void);
static void Menu_SixAxis_Exit(void);
static void Menu_Show_SixAxis(void);
static void Menu_Attitude_Enter(void);
static void Menu_Attitude_Exit(void);
static void Menu_Show_Attitude(void);
/*声明六轴进入，退出，显示函数*/

static const MenuConfig MenuTable[MENU_COUNT] = {
    [MENU_MAIN] = {"Main menu", MENU_NONE, NULL, 0, 0, 0,PAGE_TYPE_MENU},
    [MENU_SIX_AXIS_DATA] = {"Six axis data", MENU_MAIN, NULL, 0, 0, 0,PAGE_TYPE_SIX_AXIS},
    [MENU_WAVE_FILIER] = {"Wave filter", MENU_MAIN, NULL, 0, 0, 0,PAGE_TYPE_WAVE_FILIER},
    [MENU_ATTITUDE_SOLVER] = {"Attitude solver", MENU_MAIN, NULL, 0, 0, 0,PAGE_TYPE_ATTITUDE_SOLVER},
    [MENU_WAVE_SELECT_AXIS] = {"Axis",MENU_WAVE_FILIER, &g_wave_params.select_axis,0,5,1,PAGE_TYPE_MENU},
    [MENU_SEND_ENABLE] = {"Send",MENU_WAVE_FILIER,&g_wave_params.send_enable,0,1,1,PAGE_TYPE_MENU},
    [MENU_WAVE_ALPHA] = {"Alpha",MENU_WAVE_FILIER,&g_wave_params.alpha_x100,1,99,1,PAGE_TYPE_MENU},
};  //结构体数组，各参数

typedef struct MenuNode{
    const MenuConfig *info;
    struct MenuNode *parent;
    struct MenuNode *child;
    struct MenuNode *prev;
    struct MenuNode *next;
    void (*on_enter)(void);
    void (*on_exit)(void);
} MenuNode;  //结构体（页面信息，父节点，子节点，同级前节点，同级后节点，函数指针进入调用，离开调用）

static MenuNode nodes[MENU_COUNT];
static MenuNode *page;   //当前页面
static MenuNode *selected;  //选中节点
static uint8_t editing;  //是否编辑

static void Menu_BuildLinks(void){
    //清空指针
    for(uint8_t i = 0; i<MENU_COUNT;i++){
        nodes[i].info =&MenuTable[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = NULL;
        nodes[i].next = NULL;
        nodes[i].on_enter = NULL;
        nodes[i].on_exit = NULL;  
    }
    //建立链表
    for (uint8_t i = 0; i<MENU_COUNT; i++){
        MenuNode *node = &nodes[i];
        MenuNode *parent;
        if (node->info->parent_id == MENU_NONE) continue;  //无父节点，跳过
        parent = &nodes[node->info->parent_id];   //有，取父节点
        node->parent = parent;  //设置父节点

        MenuNode *first; 
        MenuNode *last;
        first = parent->child;  //取第一子节点
        /*判断是否有子节点，没有就成为第一个，有就构建环形结构*/
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
    nodes[MENU_SIX_AXIS_DATA].on_enter = Menu_SixAxis_Enter;  //进入回调
    nodes[MENU_SIX_AXIS_DATA].on_exit = Menu_SixAxis_Exit;  //离开回调
    nodes[MENU_ATTITUDE_SOLVER].on_enter = Menu_Attitude_Enter;
    nodes[MENU_ATTITUDE_SOLVER].on_exit  = Menu_Attitude_Exit;
}

int32_t edit_value = 0;   //编辑的临时变量

static void Menu_Wave_SaveToFlash(void){
    g_wave_params.magic = WAVE_PARAMS_MAGIC;  //设置魔数
    NEUQ_Flash_write_sector(WAVE_PARAMS_SECTOR,(uint8_t *)&g_wave_params,sizeof(WaveParams));  //写入flash扇区
}

static uint8_t Menu_Update(KEY_EVENT_ENUM key){
    //判断是否编辑
    if(editing == 1){
        const MenuConfig *item = selected->info;

        switch (key){
           case KEY_UP_LONG:
           case KEY_UP_SHORT:
             edit_value += item->step;  //上键长短按都加一
             break;
           case KEY_DOWN_LONG:
           case KEY_DOWN_SHORT:
             edit_value -= item->step;  //下键长短按都减一
             break;
           case KEY_FORWARD_SHORT:
             *item->value = edit_value;  //前进键短按写入全局变量，同时退出编辑
             editing = 0;

             if (selected == &nodes[MENU_WAVE_SELECT_AXIS] ||
                 selected == &nodes[MENU_SEND_ENABLE] ||
                 selected == &nodes[MENU_WAVE_ALPHA]){
                 Menu_Wave_SaveToFlash();   //如果在选择发送数据页面同时保存到flash
                 }
             if(selected == &nodes[MENU_WAVE_ALPHA]){
                NEUQ_Filter_SetAlpha((uint8_t)g_wave_params.select_axis,(float)g_wave_params.alpha_x100 / 100.0f);
             }   //如果是alpha编辑，应用给滤波器
             break;
           /*case KEY_BACK_SHORT:
             editing = 0;  
             break;*/
           default:
           return 0;
        }
        if (edit_value < item->min) edit_value = item->min;  //编辑值限幅，防止超出范围
        if (edit_value > item->max) edit_value = item->max;
        return 1;
    }
    //非编辑
    switch(key){
        case KEY_UP_SHORT:
           selected = selected->prev;  //短按上，前一节点
           break;
        case KEY_DOWN_SHORT:    
           selected = selected->next;  //短按下，后一节点
           break;
        case KEY_FORWARD_SHORT:
           if (selected->child != NULL){  //判断是否有子节点
               if(page->on_exit) page->on_exit();  //当前页离开回调
               page = selected;
               selected = selected->child;   //光标显示第一子节点
               if (page->on_enter) page->on_enter();   //进入回调
           }else if(selected->info->value != NULL){    //有参数编辑
               editing = 1;  //开始编辑
               edit_value = *selected->info->value;    //保存到临时变量
           }else if(selected->info->page_type != PAGE_TYPE_MENU){  //是否特殊节点
               if(page->on_exit) page->on_exit();
               page = selected;
               if(page->on_enter) page->on_enter();
           }else{
            return 0;
           }
           break;
        case KEY_BACK_SHORT:{
           if (page == &nodes[MENU_WAVE_FILIER] && editing == 0) {  //如果滤波页面，非编辑
                g_wave_params.select_axis = (g_wave_params.select_axis + 1) % 6;  //轴号循环
                Menu_Wave_SaveToFlash();  //保存flash
                NEUQ_Filter_SetAlpha((uint8_t)g_wave_params.select_axis,(float)g_wave_params.alpha_x100 / 100.0f);
                return 1;
            }
           if(page->parent == NULL) return 0;  //无父页
           if (page->on_exit) page->on_exit();  //离开回调
                selected = page;
                page = page->parent;
           break;
        }
        case KEY_BACK_LONG:{   //长按同样返回
           if (page->parent == NULL) return 0;
           if (page->on_exit) page->on_exit();
           selected = page;
           page = page->parent;
           return 1;
        }
        default:{
            return 0;
        } 
    }
    return 1;
}

static uint8_t six_axis_first_enter = 1;  //六轴页首次进入标志

static void Menu_SixAxis_Enter(void){  //进入
    six_axis_first_enter = 1;
}
static void Menu_SixAxis_Exit(void){   //离开

}
static void Menu_Show_SixAxis(void){   //页面显示
    char line[40];
    if (six_axis_first_enter){
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);

        NEUQ_LCD200_DisplayString(8, 8, "Six axis data");
        NEUQ_LCD200_DisplayString(8, 48, "x_accel:");
        NEUQ_LCD200_DisplayString(8, 80, "y_accel:");
        NEUQ_LCD200_DisplayString(8, 112, "z_accel:");
        NEUQ_LCD200_DisplayString(8, 144, "x_omega:");
        NEUQ_LCD200_DisplayString(8, 176, "y_omega");
        NEUQ_LCD200_DisplayString(8, 208, "z_omega");

        six_axis_first_enter = 0;

    }
    float ax = MPU6050_acc_transition(mpu6050_acc_x);  //原始值转化
    float ay = MPU6050_acc_transition(mpu6050_acc_y);
    float az = MPU6050_acc_transition(mpu6050_acc_z);
    float gx = MPU6050_gyro_transition(mpu6050_gyro_x);
    float gy = MPU6050_gyro_transition(mpu6050_gyro_y);
    float gz = MPU6050_gyro_transition(mpu6050_gyro_z);

    snprintf(line,sizeof(line),"%+.3f g",ax);  //格式化
    NEUQ_LCD200_DisplayString(96, 48, line);   //显示

    snprintf(line,sizeof(line),"%+.3f g",ay);
    NEUQ_LCD200_DisplayString(96, 80, line);

    snprintf(line,sizeof(line),"%+.3f g",az);
    NEUQ_LCD200_DisplayString(96, 112, line);

    snprintf(line,sizeof(line),"%+.3f dps  ",gx);
    NEUQ_LCD200_DisplayString(96, 144, line);

    snprintf(line,sizeof(line),"%+.3f dps  ",gy);
    NEUQ_LCD200_DisplayString(96, 176, line);

    snprintf(line,sizeof(line),"%+.3f dps  ",gz);
    NEUQ_LCD200_DisplayString(96, 208, line);
   
}

static void Menu_Wave_LoadFromFlash(void){    //加载flash参数
    WaveParams tmp;  //临时结构体
    if(NEUQ_Flash_read_sector(WAVE_PARAMS_SECTOR,(uint8_t *)&tmp,sizeof(WaveParams)) != NEUQ_FLASH_OK){
        return;   //读扇区511
    }
    if (tmp.magic != WAVE_PARAMS_MAGIC){  //检查魔数
        return;
    }
    g_wave_params = tmp;
    if(g_wave_params.select_axis < 0 || g_wave_params.select_axis > 5){  //轴号范围
        g_wave_params.select_axis = 0;
    }
    if(g_wave_params.send_enable != 0 &&g_wave_params.send_enable != 1){
        g_wave_params.send_enable = 0;
    }
    if(g_wave_params.alpha_x100 < 1|| g_wave_params.alpha_x100 >99){   //异常值恢复
        g_wave_params.alpha_x100 = 20;
    }
    NEUQ_Filter_SetAlpha((uint8_t)g_wave_params.select_axis,(float)g_wave_params.alpha_x100 / 100.0f);  //应用给滤波器
}




static uint8_t Menu_Show_List(void){
    MenuNode *item = page->child;  //从子节点开始

    NEUQ_LCD200_Clear();
    char line[32];
    uint16_t y = 48;
    snprintf(line,sizeof(line),"%s", page->info->name);  //初始化
    NEUQ_LCD200_DisplayString(8,8,line);

    if (item != NULL){   //有子节点
        do {
            char mark = ' ';
            const MenuConfig *info = item->info;

            if(item == selected){  //选定的当前页
                mark = editing?'*':'>';  //编辑用“*”，普通用“>”
                NEUQ_LCD200_SetPenColor(LCD_WHITE);  //高亮显示
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8, y, 320, 24);
            }

            if(info->value != NULL){  //有参数可编辑
                int32_t value =((item == selected)&&(editing))? edit_value : (*info->value);  //编辑显示编辑量，否则全局量
                if(item == &nodes[MENU_WAVE_SELECT_AXIS]){  //轴项
                    static const char *axis_name[6] = {
                        "x_accel","y_accel","z_accel",
                        "x_omega","y_omega","z_omega"
                    };
                    if(value < 0 ||value > 5) value = 0;  //轴号修正
                    snprintf(line,sizeof(line),"%c %s: %s",mark, info->name, axis_name[value]);
                }else if(item == &nodes[MENU_SEND_ENABLE]){  //发送选择
                    snprintf(line,sizeof(line),"%c %s: %s",mark, info->name, (value != 0) ?"ON" :"OFF");
                }else if(item == &nodes[MENU_WAVE_ALPHA]){   //alpha参数选择
                    snprintf(line,sizeof(line),"%c %s: 0.%02ld",mark,info->name,(long)value);
                }else{
                    snprintf(line,sizeof(line),"%c %s:%ld",mark,info->name,value);
                }
            }else{   //无参数
                snprintf(line,sizeof(line),"%c %s",mark,info->name);
            }
            NEUQ_LCD200_DisplayString(8, y,line);

            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y+=32;
            item = item->next;
        }while(item != page->child);  //出口条件循环，保证至少一次
    }
    return 0;
}
static uint8_t attitude_first_enter = 1;

static void Menu_Attitude_Enter(void){
    attitude_first_enter = 1;
}

static void Menu_Attitude_Exit(void){

}

static void Menu_Show_Attitude(void)
{
    char line[40];

    if (attitude_first_enter) {
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);

        NEUQ_LCD200_DisplayString(8,   8, "Attitude solver");
        NEUQ_LCD200_DisplayString(8,  64, "Roll :");
        NEUQ_LCD200_DisplayString(8, 112, "Pitch:");
        NEUQ_LCD200_DisplayString(8, 160, "Yaw  :");
        attitude_first_enter = 0;
    }

    // 用 Get 函数，触发 computeAngles，返回度数
    snprintf(line, sizeof(line), "%+.2f deg  ", NEUQ_Attitude_GetRoll());
    NEUQ_LCD200_DisplayString(88,  64, line);

    snprintf(line, sizeof(line), "%+.2f deg  ", NEUQ_Attitude_GetPitch());
    NEUQ_LCD200_DisplayString(88, 112, line);

    snprintf(line, sizeof(line), "%+.2f deg  ", NEUQ_Attitude_GetYaw());
    NEUQ_LCD200_DisplayString(88, 160, line);
}


static uint8_t Menu_Show(void){
    switch(page->info->page_type){
        case PAGE_TYPE_MENU:  //主菜单页面
            Menu_Show_List();
            break;
        case PAGE_TYPE_SIX_AXIS:  //六轴页面
            Menu_Show_SixAxis();
            break;
       case PAGE_TYPE_WAVE_FILIER:  //滤波页面
            Menu_Show_List();
            break;
        case PAGE_TYPE_ATTITUDE_SOLVER:  //姿态解算页面
            Menu_Show_Attitude(); 
            break;
    }
    return 0;
}

void NEUQ_Menu_Init(void){
    params.x_accel = 0;
    params.x_omega = 0;
    params.y_accel = 0;
    params.y_omega = 0;
    params.z_accel = 0;
    params.z_omega = 0;
    Menu_BuildLinks();
    page = &nodes[MENU_MAIN];
    selected = page->child;
    editing = 0;

    Menu_Wave_LoadFromFlash();
    
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    Menu_Show();
}   //菜单初始化

void NEUQ_Menu_Task(void){
    KEY_EVENT_ENUM key;
    key = NEUQ_Key_EventGet();  //获取按键事件

    if(key != KEY_EVENT_NONE){
        Menu_Update(key);  //有事件，处理按键事件
    }
    if(page->info->page_type == PAGE_TYPE_MENU || page->info->page_type == PAGE_TYPE_WAVE_FILIER){
        if (key != KEY_EVENT_NONE){
            Menu_Show();  //特殊页面按键刷新
        }  
    }else{
        Menu_Show();
    }

}

void NEUQ_Menu_WaveSend(void){   //向vofa+发送数据
    if(g_wave_params.send_enable == 0) return;
    float send_buf[2];   //定义发送缓冲区
    send_buf[0] = filter_raw[g_wave_params.select_axis];  //原始值
    send_buf[1] = filter_out[g_wave_params.select_axis];  //滤波值
    NEUQ_Uart_write_data(UART_0,(uint8_t *)send_buf, 8);  //共八字节信号
    uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
    NEUQ_Uart_write_data(UART_0,tail, 4);  //四字节帧尾
}
