#include "NEUQ_lib_include.h"
#include "stdint.h"
#include "NEUQ_Filter.h"
#include "NEUQ_Attitude.h"

// 全局菜单参数：后台滤波与页面显示共用同一份，保证页面状态与后台一致
NEUQ_Menu_Params params;

// 页面编号
typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_IMU,      // 六轴数据显示
    MENU_FILTER,   // 波形与滤波设置
    MENU_ATT,      // 姿态解算
    MENU_COUNT
} MenuID;

// 六个轴名，与下标 0..5 一一对应
static const char *axis_name[6] = {"acc_x","acc_y","acc_z","gyr_x","gyr_y","gyr_z"};

int8_t  selected_axis = 0;   // 当前选中/上传的轴 (0..5)
uint8_t enable_send  = 0;    // 波形上传开关

// 滤波设置页光标：0..5 对应六个轴，6 对应 Send 行
#define FILTER_CURSOR_SEND 6

// 波形上传节拍：每 20ms 发一对原始/滤波值（50Hz），避免主循环无延时导致上传过快
#define SEND_PERIOD_MS 20

// 重绘类型：切页才整屏清屏重绘，数值变化只覆盖刷新动态行，避免闪烁
typedef enum{
    REDRAW_NONE = 0,
    REDRAW_FULL,       // 切页：整页重绘
    REDRAW_DYNAMIC,    // 数值变化：只覆盖刷新动态行
} RedrawType;

// 页面信息配置
typedef struct{
    const char *name;      // 页面名称
    MenuID parent_id;      // 父页面编号
} MenuConfig;

static const MenuConfig MenuTable[MENU_COUNT] = {
    [MENU_MAIN]   = {"Main menu",  MENU_NONE},
    [MENU_IMU]    = {"IMU Data",   MENU_MAIN},
    [MENU_FILTER] = {"Filter Set", MENU_MAIN},
    [MENU_ATT]    = {"Attitude",   MENU_MAIN},
};

// 菜单链表节点
typedef struct MenuNode{
    const MenuConfig *info;    // 节点内容
    struct MenuNode *parent;   // 父节点
    struct MenuNode *child;    // 子节点
    struct MenuNode *prev;     // 同级前节点
    struct MenuNode *next;     // 同级后节点
} MenuNode;

static MenuNode nodes[MENU_COUNT];
static MenuNode *page;          // 当前页面
static MenuNode *selected;      // 一级菜单上的光标项
static int8_t   filter_cursor;  // 滤波页光标：0..5 轴，6 = Send 行
static uint8_t  editing;        // 0:浏览  1:编辑某轴滤波参数
static int8_t   edit_axis;      // 正在编辑哪个轴
static int32_t  edit_value;     // 编辑中的临时值
static int32_t  edit_backup;    // 进入编辑前的值，取消时恢复

// 根据菜单表构建节点链接关系
static void Menu_BuildLinks(void){
    for(uint8_t i = 0; i < MENU_COUNT; i++){
        nodes[i].info   = &MenuTable[i];
        nodes[i].parent = NULL;
        nodes[i].child  = NULL;
        nodes[i].prev   = NULL;
        nodes[i].next   = NULL;
    }
    for(uint8_t i = 0; i < MENU_COUNT; i++){
        MenuNode *node = &nodes[i];
        if(node->info->parent_id == MENU_NONE) continue;
        MenuNode *parent = &nodes[node->info->parent_id];
        node->parent = parent;

        MenuNode *first = parent->child;
        if(first == NULL){
            parent->child = node;
            node->prev = node;
            node->next = node;
        }else{
            MenuNode *last = first->prev;
            node->prev = last;
            node->next = first;
            last->next = node;
            first->prev = node;
        }
    }
}

// 按键处理，返回需要哪种重绘
static RedrawType Menu_Update(KEY_EVENT_ENUM key){
    if(key == KEY_EVENT_NONE) return REDRAW_NONE;

    // ---- 编辑某轴滤波参数 ----
    if(editing){
        switch(key){
            case KEY_UP_SHORT:
            case KEY_UP_LONG:                    // 长按连续加
                if(edit_value < FILTER_PARAM_MAX) edit_value += FILTER_PARAM_STEP;
                params.filter_param[edit_axis] = edit_value; // 立即生效，便于观察滤波效果
                return REDRAW_DYNAMIC;
            case KEY_DOWN_SHORT:
            case KEY_DOWN_LONG:                  // 长按连续减
                if(edit_value > FILTER_PARAM_MIN) edit_value -= FILTER_PARAM_STEP;
                params.filter_param[edit_axis] = edit_value;
                return REDRAW_DYNAMIC;
            case KEY_FORWARD_SHORT:{             // 确认：六个参数全部写入 Flash
                Menu_Flash_DataTypeDef fd = {0};
                for(uint8_t i = 0; i < 6; i++) fd.filter_param[i] = params.filter_param[i];
                fd.magic_flag = MAGIC_FLAG;
                NEUQ_Flash_save(FLASH_SAVE_ADDRESS, (uint8_t *)&fd, sizeof(fd));
                editing = 0;
                return REDRAW_DYNAMIC;
            }
            case KEY_BACK_SHORT:                 // 取消：恢复该轴原值，不写 Flash
                params.filter_param[edit_axis] = edit_backup;
                editing = 0;
                return REDRAW_DYNAMIC;
            default:
                return REDRAW_NONE;
        }
    }

    // ---- 浏览模式 ----
    switch(key){
        case KEY_UP_SHORT:
            if(page == &nodes[MENU_FILTER]){
                if(filter_cursor > 0) filter_cursor--;
                if(filter_cursor < FILTER_CURSOR_SEND) selected_axis = filter_cursor;
                return REDRAW_DYNAMIC;
            }else if(page == &nodes[MENU_MAIN]){
                selected = selected->prev;
                return REDRAW_FULL;
            }
            return REDRAW_NONE;

        case KEY_DOWN_SHORT:
            if(page == &nodes[MENU_FILTER]){
                if(filter_cursor < FILTER_CURSOR_SEND) filter_cursor++;
                if(filter_cursor < FILTER_CURSOR_SEND) selected_axis = filter_cursor;
                return REDRAW_DYNAMIC;
            }else if(page == &nodes[MENU_MAIN]){
                selected = selected->next;
                return REDRAW_FULL;
            }
            return REDRAW_NONE;

        case KEY_FORWARD_SHORT:
            if(page == &nodes[MENU_MAIN]){
                page = selected;                // 进入选中页面
                editing = 0;
                return REDRAW_FULL;
            }else if(page == &nodes[MENU_FILTER]){
                if(filter_cursor == FILTER_CURSOR_SEND){
                    enable_send = !enable_send; // Send 行：开关波形上传
                }else{
                    editing = 1;                // 进入该轴参数编辑
                    edit_axis   = filter_cursor;
                    edit_value  = params.filter_param[edit_axis];
                    edit_backup = params.filter_param[edit_axis];
                }
                return REDRAW_DYNAMIC;
            }
            return REDRAW_NONE;

        case KEY_BACK_SHORT:
            if(page != &nodes[MENU_MAIN]){
                page = page->parent;            // 返回一级菜单
                editing = 0;
                return REDRAW_FULL;
            }
            return REDRAW_NONE;

        case KEY_BACK_LONG:
            // 姿态页长按返回键：重新静置校准陀螺仪零偏（校准期间模块需保持静止）
            if(page == &nodes[MENU_ATT]){
                NEUQ_Attitude_Calibrate();
                return REDRAW_FULL;
            }
            return REDRAW_NONE;

        default:
            return REDRAW_NONE;
    }
}

// 只绘制会动态变化的数据行（不清屏，覆盖式刷新，避免整屏闪烁）
static void Menu_ShowDynamic(void){
    char line[32];
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);

    if(page == &nodes[MENU_FILTER]){
        // 六轴各自一行：光标 + 轴名 + 该轴参数 + 原始值->滤波值（16px 小字体）
        int16_t raw[6] = {
            mpu6050_acc_x, mpu6050_acc_y, mpu6050_acc_z,
            mpu6050_gyro_x, mpu6050_gyro_y, mpu6050_gyro_z,
        };
        uint16_t y = 36;
        NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
        for(uint8_t i = 0; i < 6; i++){
            float filt = (i < 3) ? NEUQ_Filter_GetAcc(i) : NEUQ_Filter_GetGyro(i - 3);
            char mark = (editing && edit_axis == i) ? '*' : ((filter_cursor == i) ? '>' : ' ');
            int32_t pv = (editing && edit_axis == i) ? edit_value : params.filter_param[i];
            snprintf(line, sizeof(line), "%c%s P:%3d %6d->%6d", mark, axis_name[i], pv, raw[i], (int32_t)filt);
            NEUQ_LCD200_DisplayString(8, y, line);
            y += 20;
        }
        // Send 行：光标移到这行按确定键开关上传
        char mark_s = (filter_cursor == FILTER_CURSOR_SEND) ? '>' : ' ';
        snprintf(line, sizeof(line), "%cSend : %-3s", mark_s, enable_send ? "ON" : "OFF");
        NEUQ_LCD200_DisplayString(8, y, line);
        y += 24;
        // 操作提示
        snprintf(line, sizeof(line), "UP/DN:move  FW:edit  BK:back");
        NEUQ_LCD200_DisplayString(8, y, line);
        NEUQ_LCD200_SetAsciiFont(&ASCII_Font24);
    }
    else if(page == &nodes[MENU_IMU]){
        // 六轴同屏动态显示物理值并注明单位（24px 字体）
        float acc_g[3] = {
            MPU6050_acc_transition(mpu6050_acc_x),
            MPU6050_acc_transition(mpu6050_acc_y),
            MPU6050_acc_transition(mpu6050_acc_z),
        };
        float gyr_d[3] = {
            MPU6050_gyro_transition(mpu6050_gyro_x),
            MPU6050_gyro_transition(mpu6050_gyro_y),
            MPU6050_gyro_transition(mpu6050_gyro_z),
        };
        uint16_t y = 48;
        for(uint8_t i = 0; i < 3; i++){
            snprintf(line, sizeof(line), "%s: %6.2f g", axis_name[i], acc_g[i]);
            NEUQ_LCD200_DisplayString(8, y, line);
            y += 28;
        }
        for(uint8_t i = 0; i < 3; i++){
            snprintf(line, sizeof(line), "%s: %6.2f deg/s", axis_name[i + 3], gyr_d[i]);
            NEUQ_LCD200_DisplayString(8, y, line);
            y += 28;
        }
    }
    else if(page == &nodes[MENU_ATT]){
        uint16_t y = 48;
        snprintf(line, sizeof(line), "Roll : %7.2f deg", roll);
        NEUQ_LCD200_DisplayString(8, y, line); y += 28;
        snprintf(line, sizeof(line), "Pitch: %7.2f deg", pitch);
        NEUQ_LCD200_DisplayString(8, y, line); y += 28;
        snprintf(line, sizeof(line), "Yaw  : %7.2f deg", yaw);
        NEUQ_LCD200_DisplayString(8, y, line); y += 28;
        snprintf(line, sizeof(line), "Hold BK: re-calibrate");
        NEUQ_LCD200_DisplayString(8, y, line);
    }
}

// 整页显示逻辑（切页时调用一次）
static void Menu_Show(void){
    NEUQ_LCD200_Clear();
    char line[32];
    uint16_t y = 48;

    // 页面标题
    snprintf(line, sizeof(line), "%s", page->info->name);
    NEUQ_LCD200_DisplayString(8, 8, line);

    if(page == &nodes[MENU_MAIN]){
        // 一级菜单：三个入口，光标用 '>' 标记
        MenuNode *item = page->child;
        if(item != NULL){
            do{
                char mark = (item == selected) ? '>' : ' ';
                snprintf(line, sizeof(line), "%c %s", mark, item->info->name);
                if(item == selected){
                    NEUQ_LCD200_SetPenColor(LCD_WHITE);
                    NEUQ_LCD200_SetBackColor(LCD_BLUE);
                    NEUQ_LCD200_ClearRect(8, y, 320, 24);
                }
                NEUQ_LCD200_DisplayString(8, y, line);
                NEUQ_LCD200_SetPenColor(LCD_BLACK);
                NEUQ_LCD200_SetBackColor(LCD_WHITE);
                y += 32;
                item = item->next;
            }while(item != page->child);
        }
    }

    // 各页的动态数据行
    Menu_ShowDynamic();
}

void NEUQ_Menu_Init(void){
    Menu_BuildLinks();
    page          = &nodes[MENU_MAIN];
    selected      = page->child;
    editing       = 0;
    filter_cursor = 0;
    edit_axis     = 0;

    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();

    Menu_Show();  // 上电刷新一次菜单画面
}

void NEUQ_Menu_Task(void){
    KEY_EVENT_ENUM key = NEUQ_Key_EventGet();
    RedrawType r = Menu_Update(key);

    // 切页 -> 整页重绘；数值变化 -> 立即覆盖刷新；否则定时覆盖刷新动态数据行
    static uint32_t last_refresh = 0;
    uint32_t now = NEUQ_Get_Timer_Value_ms();
    if(r == REDRAW_FULL){
        Menu_Show();
        last_refresh = now;
    }else if(r == REDRAW_DYNAMIC){
        Menu_ShowDynamic();
        last_refresh = now;
    }else if((now - last_refresh) >= 50){
        Menu_ShowDynamic();
        last_refresh = now;
    }

    // 波形上传：任意时刻只发送当前选中轴，原始值 + 滤波值成对（同一次采样、同一单位）
    // 按 SEND_PERIOD_MS 节拍发送，波形在 VOFA+ 里以稳定节奏滚动，便于对比
    if(enable_send){
        static uint32_t last_send = 0;
        if((now - last_send) >= SEND_PERIOD_MS){
            last_send = now;
            int16_t raw;
            float   filt;
            if(selected_axis < 3){
                const int16_t raw_acc[3] = {mpu6050_acc_x, mpu6050_acc_y, mpu6050_acc_z};
                raw  = raw_acc[selected_axis];
                filt = NEUQ_Filter_GetAcc(selected_axis);
            }else{
                const int16_t raw_gyro[3] = {mpu6050_gyro_x, mpu6050_gyro_y, mpu6050_gyro_z};
                raw  = raw_gyro[selected_axis - 3];
                filt = NEUQ_Filter_GetGyro(selected_axis - 3);
            }
            printf("%d,%.2f\r\n", raw, filt);
        }
    }
}
