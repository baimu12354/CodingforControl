/********************************************************************************
 * 文件    : Key.c
 * 功能    : 按键扫描 + 二级菜单 + 页面绘制（界面交互层）
 * 说明    : 1. 从 Task_Gyro.c 拆分出来，负责按键/菜单/绘制
 *           2. 依赖 Task_Gyro.c 提供的数据(cfg/raw)和 Cfg_Save()、Task_Gyro_Calibrate()；
 *              姿态角来自 Attitude.c(att_roll/att_pitch/att_yaw)
 *           3. 代码里所有【需修改】标注，都是需要按你的硬件/接线调整的地方
 ********************************************************************************/
#include "NEUQ_lib_include.h"
#include <math.h>
#include "Key.h"
#include "Task_Gyro.h"
#include "Attitude.h"

/*=========================== 【需修改】区：按键引脚 ===========================*/
// 4 个独立按键引脚（低电平有效：按键一端接引脚、另一端接地，引脚配内部上拉）
#define KEY_UP_PIN    PE8
#define KEY_DOWN_PIN  PE9
#define KEY_OK_PIN    PE7      // 确认键
#define KEY_BACK_PIN  PE10      // 返回键
/*=========================== 【需修改】区结束 ===========================*/

// 按键下标
#define KEY_UP    0
#define KEY_DOWN  1
#define KEY_OK    2
#define KEY_BACK  3

/*=============================== 全局变量 ===============================*/
uint8_t page = PAGE_MENU;              // 当前页面（Task_Gyro.c 采样判断用）
static uint8_t menu_cursor   = 0;      // 主菜单光标
static uint8_t filter_cursor = 0;      // 滤波设置页光标：0=选轴 1=发送开关 2=滤波系数
static uint8_t edit_mode     = 0;      // 是否处于"编辑数值"状态
static uint8_t page_dirty    = 1;      // 需要完整重绘页面
static uint32_t last_refresh = 0;      // 上次动态刷新时间

static const GPIO_Ports_ENUM Key_Pins[4] = {KEY_UP_PIN, KEY_DOWN_PIN, KEY_OK_PIN, KEY_BACK_PIN};
static uint8_t key_up[4] = {1,1,1,1};

/*=============================== 菜单表 ===============================*/

// 菜单项结构：显示文字 + 选中后进入的页面
typedef struct {
    const char *name;     // 菜单项文字
    uint8_t     page;     // 目标页面编号
} MenuItem_t;

// 主菜单表（一张表描述整个菜单，加/删/调顺序只改这里）
static const MenuItem_t menu_items[MENU_NUM] = {
    {"1.Six-Axis Data", PAGE_DISPLAY},   // 0
    {"2.Wave Filter Set", PAGE_FILTER},  // 1
    {"3.Attitude Solve", PAGE_ATTITUDE}, // 2
};

/*=============================== 基础工具 ===============================*/

// 浮点数转字符串（不用 printf %f，兼容 microlib），dec 为小数位数
static void Float2Str(float v, uint8_t dec, char *out){
    if(v < 0){ *out++ = '-'; v = -v; }
    int32_t ipart = (int32_t)v;
    float frac = v - (float)ipart;

    // 整数部分（倒序放入临时数组再翻转）
    char tmp[16];
    int idx = 0;
    if(ipart == 0){ tmp[idx++] = '0'; }
    else{
        while(ipart){ tmp[idx++] = (char)('0' + ipart % 10); ipart /= 10; }
    }
    while(idx) *out++ = tmp[--idx];

    // 小数部分
    *out++ = '.';
    for(uint8_t i = 0; i < dec; i++){
        frac *= 10.0f;
        uint8_t d = (uint8_t)frac;
        *out++ = (char)('0' + d);
        frac -= (float)d;
    }
    *out = '\0';
}

// 在固定宽度位置显示文本：不足 len 位补空格，用于无残留刷新
static void LCD_Text(uint16_t x, uint16_t y, const char *s, uint8_t len){
    char buf[32];
    uint8_t i;
    for(i = 0; i < len; i++) buf[i] = ' ';
    for(i = 0; s[i] && i < len; i++) buf[i] = s[i];
    buf[len] = '\0';
    NEUQ_LCD200_DisplayString(x, y, buf);
}

// 在固定宽度位置显示浮点数（右对齐补空格）
static void LCD_Float(uint16_t x, uint16_t y, float v, uint8_t len, uint8_t dec){
    char num[32], buf[32];
    Float2Str(v, dec, num);
    uint8_t n = (uint8_t)strlen(num);
    if(n > len) n = len;
    uint8_t pad = len - n;
    for(uint8_t i = 0; i < pad; i++) buf[i] = ' ';
    for(uint8_t i = 0; i < n; i++) buf[pad + i] = num[i];
    buf[len] = '\0';
    NEUQ_LCD200_DisplayString(x, y, buf);
}

/*=============================== 按键扫描 ===============================*/

// 读一次按键：只在"按下沿"返回1（带20ms消抖，按一次只触发一次）
static uint8_t Key_Pressed(uint8_t idx){
    GPIO_Ports_ENUM pin = Key_Pins[idx];
    if(key_up[idx] && NEUQ_GPIO_read(pin) == GPIO_Low){   // 检测到按下
        NEUQ_Delay_ms(20);                                 // 消抖
        if(NEUQ_GPIO_read(pin) == GPIO_Low){
            key_up[idx] = 0;
            return 1;
        }
    }
    if(NEUQ_GPIO_read(pin) == GPIO_High){                  // 松开后才允许下次触发
        key_up[idx] = 1;
    }
    return 0;
}

/*=============================== 菜单/按键逻辑 ===============================*/

static void Handle_Keys(uint8_t up, uint8_t down, uint8_t ok, uint8_t back){
    // 返回键：非菜单页 -> 返回菜单；编辑态 -> 先退出编辑
    if(back){
        if(page != PAGE_MENU){
            if(edit_mode) edit_mode = 0;
            else          page = PAGE_MENU;
            page_dirty = 1;
        }
        return;                     // 一次只处理一个键
    }

    if(page == PAGE_MENU){
        if(up){                              // 上移：到顶则跳到最后一个（头尾相连）
            menu_cursor = (menu_cursor > 0) ? menu_cursor - 1 : MENU_NUM - 1;
            page_dirty = 1;
        }
        if(down){                            // 下移：到底则跳回第一个（头尾相连）
            menu_cursor = (menu_cursor < MENU_NUM - 1) ? menu_cursor + 1 : 0;
            page_dirty = 1;
        }
        if(ok){
            page = menu_items[menu_cursor].page;   // 查表跳转
            page_dirty = 1;
        }
    }
    else if(page == PAGE_FILTER){
        if(edit_mode){                      // 正在编辑数值
            if(up || down){
                int8_t d = up ? 1 : -1;
                if(filter_cursor == 0){     // 选轴：0~5 循环
                    int16_t v = (int16_t)cfg.sel_axis + d;
                    if(v < 0)          v = AXIS_NUM - 1;
                    if(v >= AXIS_NUM)  v = 0;
                    cfg.sel_axis = (uint8_t)v;
                }
                else if(filter_cursor == 1){ // 发送开关：直接取反
                    cfg.send_on = !cfg.send_on;
                }
                else{                        // 滤波系数：0.01 步进，范围 0.01~1.00
                    float v = cfg.alpha + d * 0.01f;
                    if(v < 0.01f) v = 0.01f;
                    if(v > 1.00f) v = 1.00f;
                    cfg.alpha = v;
                }
                page_dirty = 1;
            }
            if(ok){                          // 确认 -> 退出编辑并保存到 Flash
                edit_mode = 0;
                Cfg_Save();
                page_dirty = 1;
            }
        }
        else{                                // 非编辑：移动光标
            if(up   && filter_cursor > 0) { filter_cursor--; page_dirty = 1; }
            if(down && filter_cursor < 2) { filter_cursor++; page_dirty = 1; }
            if(ok){
                if(filter_cursor == 1){      // 发送开关直接切换，不用进入编辑
                    cfg.send_on = !cfg.send_on;
                    Cfg_Save();
                }
                else{
                    edit_mode = 1;           // 选轴/系数 进入编辑
                }
                page_dirty = 1;
            }
        }
    }
    else if(page == PAGE_ATTITUDE){
        if(ok){                              // 姿态页按 OK：重新静置校准（零偏 + 初始姿态 + 航向归零）
            NEUQ_LCD200_Clear();
            NEUQ_LCD200_DisplayString(50, 100, "Gyro Calibrating...");
            NEUQ_LCD200_DisplayString(60, 130, "Keep Module Still");
            Task_Gyro_Calibrate();
            page_dirty = 1;                  // 校准完成后重绘姿态页
        }
    }
}

/*=============================== 各页面绘制 ===============================*/

static void Draw_Menu(void){
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(60, 20, "FUNCTION MENU");

    for(uint8_t i = 0; i < MENU_NUM; i++){
        uint16_t y = 70 + i * 40;
        LCD_Text(40, y, (i == menu_cursor) ? ">" : " ", 1);
        NEUQ_LCD200_DisplayString(60, y, (char *)menu_items[i].name);
    }
}

static void Draw_Display_Static(void){
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(70, 10, "SIX-AXIS DATA");

    NEUQ_LCD200_DisplayString(20,  45, "Accel(g):");
    NEUQ_LCD200_DisplayString(20,  65, "Ax:");
    NEUQ_LCD200_DisplayString(20,  85, "Ay:");
    NEUQ_LCD200_DisplayString(20, 105, "Az:");

    NEUQ_LCD200_DisplayString(20, 140, "Gyro(deg/s):");
    NEUQ_LCD200_DisplayString(20, 160, "Gx:");
    NEUQ_LCD200_DisplayString(20, 180, "Gy:");
    NEUQ_LCD200_DisplayString(20, 200, "Gz:");

    LCD_Text(20, 225, "Send:", 5);
}

static void Draw_Display_Dynamic(void){
    LCD_Float(80,  65, raw[AXIS_AX], 8, 2);
    LCD_Float(80,  85, raw[AXIS_AY], 8, 2);
    LCD_Float(80, 105, raw[AXIS_AZ], 8, 2);
    LCD_Float(80, 160, raw[AXIS_GX], 8, 2);
    LCD_Float(80, 180, raw[AXIS_GY], 8, 2);
    LCD_Float(80, 200, raw[AXIS_GZ], 8, 2);

    // 底部实时显示上传状态（与后台一致）
    LCD_Text(110, 225, Axis_Names[cfg.sel_axis], 2);
    LCD_Text(150, 225, cfg.send_on ? "ON " : "OFF", 3);
}

static void Draw_Filter_Dynamic(void){
    // 光标
    LCD_Text(10, 50,  (filter_cursor == 0) ? ">" : " ", 1);
    LCD_Text(10, 80,  (filter_cursor == 1) ? ">" : " ", 1);
    LCD_Text(10, 110, (filter_cursor == 2) ? ">" : " ", 1);

    // 值
    LCD_Text(120, 50,  Axis_Names[cfg.sel_axis], 2);
    LCD_Text(120, 80,  cfg.send_on ? "ON " : "OFF", 3);
    LCD_Float(120, 110, cfg.alpha, 6, 2);

    // 状态提示
    LCD_Text(20, 220, edit_mode ? "EDITING..." : "           ", 11);
}

static void Draw_Filter(void){
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(60, 10, "WAVE FILTER SET");

    NEUQ_LCD200_DisplayString(20,  50, "Axis:");
    NEUQ_LCD200_DisplayString(20,  80, "Send:");
    NEUQ_LCD200_DisplayString(20, 110, "Alpha:");

    NEUQ_LCD200_DisplayString(20, 160, "OK:edit/save");
    NEUQ_LCD200_DisplayString(20, 180, "UP/DOWN:change");
    NEUQ_LCD200_DisplayString(20, 200, "BACK:return");

    Draw_Filter_Dynamic();
}

static void Draw_Attitude_Static(void){
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(90, 10, "ATTITUDE");

    NEUQ_LCD200_DisplayString(20,  55, "Roll :");
    NEUQ_LCD200_DisplayString(20,  85, "Pitch:");
    NEUQ_LCD200_DisplayString(20, 115, "Yaw  :");

    NEUQ_LCD200_DisplayString(20, 160, "Quaternion 100Hz");
    NEUQ_LCD200_DisplayString(20, 185, "unit: deg (yaw rel)");
    NEUQ_LCD200_DisplayString(20, 215, "OK:Recalibrate");
}

static void Draw_Attitude_Dynamic(void){
    // 四元数解算得到的欧拉角（Attitude.c 每 10ms 更新一次），单位：度
    LCD_Float(90,  55, att_roll,  7, 1);
    LCD_Float(90,  85, att_pitch, 7, 1);
    LCD_Float(90, 115, att_yaw,   7, 1);
}

/*=============================== 初始化 ===============================*/

void Key_Init(void){
    // 1. 按键引脚：上拉输入
    NEUQ_GPIO_init(KEY_UP_PIN,   GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_DOWN_PIN, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_OK_PIN,   GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_BACK_PIN, GPIO_Mode_Input_PullUp, GPIO_High);

    // 2. 屏幕 + 字体（16x8 小字体，适合 320x240 横屏）
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);

    last_refresh = HAL_GetTick();
}

/*=============================== 主循环 ===============================*/

void Key_Loop(void){
    // 1. 读按键（每个键只在按下沿返回一次）
    uint8_t k_up   = Key_Pressed(KEY_UP);
    uint8_t k_down = Key_Pressed(KEY_DOWN);
    uint8_t k_ok   = Key_Pressed(KEY_OK);
    uint8_t k_back = Key_Pressed(KEY_BACK);

    // 2. 处理按键，更新页面状态
    Handle_Keys(k_up, k_down, k_ok, k_back);

    // 3. 页面完整重绘（切页 / 光标移动 / 数值改动时）
    if(page_dirty){
        page_dirty = 0;
        switch(page){
            case PAGE_MENU:     Draw_Menu();            break;
            case PAGE_DISPLAY:  Draw_Display_Static();  break;
            case PAGE_FILTER:   Draw_Filter();          break;
            case PAGE_ATTITUDE: Draw_Attitude_Static(); break;
        }
    }

    // 4. 动态数值刷新（每 100ms，固定宽度覆盖，不闪烁）
    uint32_t now = HAL_GetTick();
    if(now - last_refresh >= 100){
        last_refresh = now;
        if(page == PAGE_DISPLAY)       Draw_Display_Dynamic();
        else if(page == PAGE_ATTITUDE) Draw_Attitude_Dynamic();
    }
}
