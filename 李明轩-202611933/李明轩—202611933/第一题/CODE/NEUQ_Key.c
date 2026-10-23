/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_Key
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "NEUQ_Key.h"

/*
 * 单页菜单说明：
 *   - 菜单选项用双向循环链表组织：
 *       light1 ~ light4（每个选项只点亮一个对应的灯）
 *   - 上/下按键移动光标（待执行方案），确认键把待执行方案设为当前方案
 *   - 页面同时显示：当前方案、待执行方案、PA3 采集的电压值
 *
 * 注意：各方案的具体实现（LED_Scheme_Light1 ~ Light4）
 *       由用户另起文件实现，本文件仅通过函数指针调用它们。
 */

/* ==================== 按键引脚配置（请按实际接线修改） ==================== */
#define KEY_UP_PIN   PE10     // 上翻 / 上一个选项
#define KEY_DOWN_PIN PE8     // 下翻 / 下一个选项
#define KEY_OK_PIN   PE7    // 确认，执行待执行方案

#define MENU_ITEM_NUM  4     // 菜单选项数量（light1~light4）

/* ==================== 显示布局（横屏 320 x 240） ==================== */
#define TITLE_X        8
#define TITLE_Y        8

#define ITEM_X         32
#define ITEM_Y0        44
#define ITEM_DY        22
#define CURSOR_X       10
#define CUR_MARK_X     224

#define STATUS_LABEL_X 12
#define STATUS_VAL_X   96
#define STATUS_CUR_Y   150
#define STATUS_PEND_Y  172
#define STATUS_VOLT_Y  194
#define VOLT_CLR_W     160     // 电压值刷新时清除的宽度

/* ==================== 全局变量 ==================== */
static Menu_Node  g_menu[MENU_ITEM_NUM];  // 4 个链表节点（静态数组）
static Menu_Node *g_head    = NULL;       // 链表头
static Menu_Node *g_current = NULL;       // 当前方案
static Menu_Node *g_pending = NULL;       // 待执行方案（光标所在）

/* ==================== 按键扫描（带 20ms 消抖，返回新按下的按键） ==================== */
#define KEY_UP    0x01
#define KEY_DOWN  0x02
#define KEY_OK    0x04

static uint8_t Key_Scan(void){
    static uint32_t last_tick = 0;
    static uint8_t  last_key  = 0;

    if(HAL_GetTick() - last_tick < 20)  return 0;    // 20ms 消抖
    last_tick = HAL_GetTick();

    uint8_t key = 0;
    if(NEUQ_GPIO_read(KEY_UP_PIN)   == GPIO_Low)  key |= KEY_UP;
    if(NEUQ_GPIO_read(KEY_DOWN_PIN) == GPIO_Low)  key |= KEY_DOWN;
    if(NEUQ_GPIO_read(KEY_OK_PIN)   == GPIO_Low)  key |= KEY_OK;

    uint8_t pressed = key & (~last_key);             // 只取“新按下”的键
    last_key = key;
    return pressed;
}

/* ==================== 链表菜单 ==================== */

// 构建双向循环链表，并把 4 个节点链接起来
static void Menu_BuildList(void){
    static const char *names[MENU_ITEM_NUM]   = {"light1", "light2", "light3", "light4"};
    // 【连接点③ 绑定】把 4 个选项名与 4 个 LED 动作函数（NEUQ_LED.c 实现）一一绑定
    static void (*schemes[MENU_ITEM_NUM])(void) = {LED_Scheme_Light1, LED_Scheme_Light2,
                                                   LED_Scheme_Light3, LED_Scheme_Light4};

    // 1. 填入节点数据
    for(uint8_t i = 0; i < MENU_ITEM_NUM; i++){
        strcpy(g_menu[i].name, names[i]);
        g_menu[i].id     = i + 1;
        g_menu[i].scheme = schemes[i];
    }

    // 2. 链接成双向循环链表（首尾相接，光标可以循环移动）
    for(uint8_t i = 0; i < MENU_ITEM_NUM; i++){
        g_menu[i].prev = &g_menu[(i + MENU_ITEM_NUM - 1) % MENU_ITEM_NUM];
        g_menu[i].next = &g_menu[(i + 1) % MENU_ITEM_NUM];
    }

    // 3. 初始状态：默认 light1 为当前方案，光标也停在 light1
    g_head    = &g_menu[0];
    g_current = &g_menu[0];
    g_pending = &g_menu[0];
}

static void Menu_MoveUp(void)    { g_pending = g_pending->prev; }   // 光标上移
static void Menu_MoveDown(void)  { g_pending = g_pending->next; }   // 光标下移
static void Menu_Confirm(void)   { g_current = g_pending; }         // 【连接点④ 触发】按下确认键后，待执行方案变为当前方案

/* ==================== 显示 ==================== */

// 采集 PA3 电压并刷新显示（单独刷新，避免每次整页重画）
static void Menu_DisplayVoltage(void){
    // 12 位 ADC：电压 = 采样值 * 3.3 / 4095
    float voltage = (float)adc_mean_filter_convert(ADC1_CH3_PA3, 8) * 3.3f / 4095.0f;
    char  buf[16];
    sprintf(buf, "%.3f V", voltage);
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_ClearRect(STATUS_VAL_X, STATUS_VOLT_Y, VOLT_CLR_W, 8);   // 清除旧值
    NEUQ_LCD200_DisplayString(STATUS_VAL_X, STATUS_VOLT_Y, buf);
}

// 整页刷新：标题 + 4 个选项 + 当前/待执行方案 + 电压
static void Menu_Display(void){
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_Clear();

    // 标题
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font24);
    NEUQ_LCD200_DisplayString(TITLE_X, TITLE_Y, "LED MENU");

    // 选项列表
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
    Menu_Node *p = g_head;
    uint16_t   y = ITEM_Y0;
    for(uint8_t i = 0; i < MENU_ITEM_NUM; i++){
        if(p == g_pending){                          // 光标指向待执行方案
            NEUQ_LCD200_DisplayString(CURSOR_X, y, ">");
        }
        NEUQ_LCD200_DisplayString(ITEM_X, y, p->name);
        if(p == g_current){                          // 标记当前方案
            NEUQ_LCD200_DisplayString(CUR_MARK_X, y, "[CUR]");
        }
        p = p->next;
        y += ITEM_DY;
    }

    // 状态区
    NEUQ_LCD200_DisplayString(STATUS_LABEL_X, STATUS_CUR_Y,  "CUR :");
    NEUQ_LCD200_DisplayString(STATUS_VAL_X,   STATUS_CUR_Y,  g_current->name);
    NEUQ_LCD200_DisplayString(STATUS_LABEL_X, STATUS_PEND_Y, "PEND:");
    NEUQ_LCD200_DisplayString(STATUS_VAL_X,   STATUS_PEND_Y, g_pending->name);
    NEUQ_LCD200_DisplayString(STATUS_LABEL_X, STATUS_VOLT_Y, "PA3 :");

    Menu_DisplayVoltage();
}

/* ==================== 对外接口 ==================== */

void Menu_Init(void){
    // 1. 初始化 LCD
    NEUQ_LCD200_init();

    // 2. 初始化 3 个按键引脚（上拉输入，按下为低电平）
    NEUQ_GPIO_init(KEY_UP_PIN,   GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_DOWN_PIN, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_OK_PIN,   GPIO_Mode_Input_PullUp, GPIO_High);

    // 3. 初始化 PA3 为 ADC 输入（12 位分辨率）
    adc_init(ADC1_CH3_PA3, ADC_12BIT);

    // 4. 构建链表菜单
    Menu_BuildList();

    // 5. 首次整页显示
    Menu_Display();
}

void Menu_Task(void){
    uint8_t key     = Key_Scan();
    uint8_t changed = 0;

    // 处理按键：移动光标 / 确认执行
    if(key & KEY_UP)   { Menu_MoveUp();   changed = 1; }
    if(key & KEY_DOWN) { Menu_MoveDown(); changed = 1; }
    if(key & KEY_OK)   { Menu_Confirm();  changed = 1; }

    if(changed){
        Menu_Display();                      // 状态变化时整页刷新
    }

    // 【连接点⑤ 执行】调用当前方案对应的 LED 动作函数（函数指针见②，绑定关系见③）
    g_current->scheme();

    // 周期性刷新电压显示（约每 300ms）
    static uint32_t last_vol = 0;
    if(HAL_GetTick() - last_vol >= 300){
        last_vol = HAL_GetTick();
        Menu_DisplayVoltage();
    }
}
