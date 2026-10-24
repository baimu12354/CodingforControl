#include"NEUQ_lib_include.h"
#include<stdint.h>
 MENU_NODE nodes[MENU_COUNT];//存储所有节点，nodes[i]就是一个节点
 MENU_NODE *page;//当前页面的节点
 MENU_NODE *select;//选择高亮项的节点
 uint8_t editing;//0->浏览模式，1->编辑模式
 float edit_temp;//用于临时存储编辑模式下更改变量的值
 MENU_ACCANDGYR acgy_raw;//六轴变量结构体存没有滤波的原始数据
 MENU_ACCANDGYR acandgy;//六轴变量结构体存储要显示的变量
 MENU_CALCULA attitude;//姿态解算结构体存储要显示的变量
// ==================== 菜单配置表 ====================
static const Menuconfig menu_table[MENU_COUNT]={
    {"MAIN",                     MENU_NONE,             MENU_MAIN,              NULL,                0,0,0},
    {"MENU_DATA_DISPLAY",        MENU_MAIN,             MENU_DATA_DISPLAY,      NULL,                0,0,0},
    {"MENU_X_ACCEL",             MENU_DATA_DISPLAY,     MENU_X_ACCEL,           &(acandgy.x_accel),  0,0,0},
    {"MENU_Y_ACCEL",             MENU_DATA_DISPLAY,     MENU_X_ACCEL,           &(acandgy.y_accel),  0,0,0},
    {"MENU_Z_ACCEL",             MENU_DATA_DISPLAY,     MENU_X_ACCEL,           &(acandgy.z_accel),  0,0,0},
    {"MENU_X_GYRO",              MENU_DATA_DISPLAY,     MENU_X_GYRO,            &(acandgy.x_gyro),   0,0,0},
    {"MENU_Y_GYRO",              MENU_DATA_DISPLAY,     MENU_X_GYRO,            &(acandgy.y_gyro),   0,0,0},
    {"MENU_Z_GYRO",              MENU_DATA_DISPLAY,     MENU_X_GYRO,            &(acandgy.z_gyro),   0,0,0},
    {"MENU_WAVEFORM_SET",        MENU_MAIN,             MENU_WAVEFORM_SET,      NULL,                0,0,0},
    {"MENU_WAVE_SENT",           MENU_WAVEFORM_SET,     MENU_WAVE_SENT,         NULL,                0,0,0},
    {"MENU_WAVE_SENT_X_ACC",     MENU_WAVE_SENT,        MENU_WAVE_SENT_X_ACC,   NULL,                0,0,0},
    {"MENU_WAVE_SENT_Y_ACC",     MENU_WAVE_SENT,        MENU_WAVE_SENT_Y_ACC,   NULL,                0,0,0},
    {"MENU_WAVE_SENT_Z_ACC",     MENU_WAVE_SENT,        MENU_WAVE_SENT_Z_ACC,   NULL,                0,0,0},
    {"MENU_WAVE_SENT_X_GYR",     MENU_WAVE_SENT,        MENU_WAVE_SENT_X_GYR,   NULL,                0,0,0},
    {"MENU_WAVE_SENT_Y_GYR",     MENU_WAVE_SENT,        MENU_WAVE_SENT_Y_GYR,   NULL,                0,0,0},
    {"MENU_WAVE_SENT_Z_GYR",     MENU_WAVE_SENT,        MENU_WAVE_SENT_Z_GYR,   NULL,                0,0,0},
    {"MENU_WAVEBL_SET",          MENU_WAVEFORM_SET,     MENU_WAVEVAARIABLE_SET, NULL,                0,0,0},
    {"WAVE_SET_X_ACC",           MENU_WAVEVAARIABLE_SET,WAVE_SET_X_ACC,         &(waveset_x_acc),    1.0f,0.0f,0.05f},
    {"WAVE_SET_Y_ACC",           MENU_WAVEVAARIABLE_SET,WAVE_SET_Y_ACC,         &(waveset_y_acc),    1.0f,0.0f,0.05f},
    {"WAVE_SET_Z_ACC",           MENU_WAVEVAARIABLE_SET,WAVE_SET_Z_ACC,         &(waveset_z_acc),    1.0f,0.0f,0.05f},
    {"WAVE_SET_X_GYR",           MENU_WAVEVAARIABLE_SET,WAVE_SET_X_GYR,         &(waveset_x_gyr),    1.0f,0.0f,0.05f},
    {"WAVE_SET_Y_GYR",           MENU_WAVEVAARIABLE_SET,WAVE_SET_Y_GYR,         &(waveset_y_gyr),    1.0f,0.0f,0.05f},
    {"WAVE_SET_Z_GYR",           MENU_WAVEVAARIABLE_SET,WAVE_SET_Z_GYR,         &(waveset_z_gyr),    1.0f,0.0f,0.05f},
    {"MENU_ATTITUDE_CALCULA",    MENU_MAIN,             MENU_ATTITUDE_CALCULA,  NULL,                0,0,0},
    {"MENU_ROLL",                MENU_ATTITUDE_CALCULA, MENU_ROLL,              &(attitude.roll),    0,0,0},
    {"MENU_PITCH",               MENU_ATTITUDE_CALCULA, MENU_PITCH,             &(attitude.pitch),   0,0,0},
    {"MENU_YAW",                 MENU_ATTITUDE_CALCULA, MENU_YAW,               &(attitude.yaw),     0,0,0}
};

// ==================== 内部函数：构建链表 ====================
static void Build_links(void)
{
    // 初始化所有节点
    for(uint16_t i=0; i<MENU_COUNT; i++){
        nodes[i].info   = &menu_table[i];
        nodes[i].parent = NULL;
        nodes[i].child  = NULL;
        nodes[i].prev   = NULL;
        nodes[i].next   = NULL;
    }

    // 构建层级关系 + 同级双向循环链表
    for (uint16_t i = 0; i < MENU_COUNT; i++)
    {
        MENU_NODE *node = &nodes[i];
        if (node->info->parent_id == MENU_NONE) continue;
        
        MENU_NODE *parent = &nodes[node->info->parent_id];
        node->parent = parent;

        MENU_NODE *first = parent->child;
        if (first == NULL)
        {
            parent->child = node;
            node->prev = node;
            node->next = node;
        }
        else
        {
            MENU_NODE *last = first->prev;
            node->prev   = last;
            last->next   = node;
            node->next   = first;
            first->prev  = node;
        }
    }
}

// ==================== 菜单更新逻辑 ====================
uint8_t Menu_Update(KEY_EVENT_ENUM Key)
{
    // 编辑模式（当前菜单项无参数，暂不会进入）
    if(editing == 1){
        const Menuconfig *item = select->info;
        switch(Key){
            case KEY_UP_SHORT:
                break;
            case KEY_DOWN_SHORT:
                break;
            case KEY_FORWORD_SHORT:
                *(item->value) = edit_temp;
                WaveSet_Save();
                editing = 0;
                break;
            case KEY_BACK_SHORT:
                editing = 0;
                break;
            case KEY_UP_LONG:
                edit_temp += item->step;
                break;  
            case KEY_DOWN_LONG:
                edit_temp -= item->step;
                break;
            default:
                return 0;
        }
        if(edit_temp < item->min) edit_temp = item->min;
        if(edit_temp > item->max) edit_temp = item->max;
        return 1;
    }

    // 浏览模式
    else{
        switch(Key){
            case KEY_UP_SHORT:
                select = select->prev;
                break;
            case KEY_DOWN_SHORT:
                select = select->next;
                break;

            case KEY_FORWORD_SHORT:
                // 有子菜单：进入子页面
                if(select->child != NULL){
                    page = select;
                    select = select->child;
                }
                // 有参数：进入编辑模式
                else if(select->info->value != NULL){
                    editing = 1;
                    edit_temp = *(select->info->value);
                }
                // 无子菜单，无参数->确定项,用于确定向VOFA+发送哪一轴的数据
                else{
                    SendToVOFA();
                }
                break;

            case KEY_BACK_SHORT:
                if(page->parent == NULL) return 0;
            // 如果当前页面是波形发送选择页，按下返回，则停止VOFA发送
            if(page->info->self_id == MENU_WAVE_SENT)
            {
              WaveSend_Stop(); //调用接口，内部把current_send_ch = MENU_NONE
            }
                select = page;
                page = page->parent;
                break;

            default:
                return 0;
        }
        return 1;
    }
}

// ==================== 菜单初始化 ====================
void Menu_Init(void)
{
    // 硬件初始化
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();

    // 构建菜单链表
    Build_links();

    // 初始页面：MAIN
    page   = &nodes[MENU_MAIN];
    select = page->child;
    editing = 0;

    // 首次刷新屏幕
    Menu_Show();
}

// ==================== 屏幕显示逻辑 ====================
void Menu_Show(void)
{
    MENU_NODE *item = page->child;
    NEUQ_LCD200_Clear();
    char line[32];
    uint16_t y = 48;

    // 显示页面标题
    snprintf(line, sizeof(line), "%s", page->info->name);
    NEUQ_LCD200_DisplayString(8, 8, line);

    if (item != NULL)
    {
        do
        {
            char mark = ' ';
            const Menuconfig *info = item->info;

            // 光标选中标记
            if (item == select) {
                mark = '>';
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8, y, 280, 24);
            }

            //其他的定义
            /*if (item == &nodes[其他])
            {
                
            }*/

            // 显示菜单项
            if (info->value != NULL)
            {
                float value = ((item == select) && (editing)) ? edit_temp : (*info->value);
                snprintf(line, sizeof(line), "%c %s:%.2f", mark, info->name, value);
            }
            else
            {
                snprintf(line, sizeof(line), "%c %s", mark, info->name);
            }
            NEUQ_LCD200_DisplayString(8, y, line);

            // 恢复默认颜色
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);

            y += 32;
            item = item->next;
        } while (item != page->child);
    }
}

// ==================== 菜单任务封装 ====================
void Menu_Task(void)
{
    KEY_EVENT_ENUM Key = NEUQ_Key_Event();
    if(Menu_Update(Key)){
        Menu_Show();
    }
}
// ===============数据刷新时只更新数据不更新页面===========
void Menu_Show_Updata_Data(void)
{
    if (page == NULL) return;

    MENU_NODE *item = page->child;
    if (item == NULL) return;   // 当前页没有子项，直接返回

    char line[32];
    uint16_t y = 48;

    do
    {
        const Menuconfig *info = item->info;

        // 只刷新有数据值的项
        if (info->value != NULL)
        {
            char mark = ' ';

            // 根据选中状态，用相应的背景色先清掉该行
            if (item == select) {
                mark = '>';
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
            } else {
                NEUQ_LCD200_SetPenColor(LCD_BLACK);
                NEUQ_LCD200_SetBackColor(LCD_WHITE);
            }
            NEUQ_LCD200_ClearRect(8, y, 280, 24);   // 只清这一行

            // 组装新内容（编辑模式下显示 edit_temp）
            float value = ((item == select) && (editing)) ? edit_temp : (*info->value);
            snprintf(line, sizeof(line), "%c %s:%.2f", mark, info->name, value);
            NEUQ_LCD200_DisplayString(8, y, line);

            // 恢复默认颜色
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
        }

        y += 32;
        item = item->next;
    } while (item != page->child);
}