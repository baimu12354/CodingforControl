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
* 文件名称          NEUQ_Main
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Main.h"
#include "LCD.h"

/*  第一题 LED 时序控制
 *
 *  LED 接 PA8，四套方案每套都是 2s 一个周期。
 *  时序状态机放在 1ms 定时器中断里跑，主循环只管按键和刷屏，
 *  所以按键、长按、刷新页面都不会把正在跑的动作打断。
 *  按键只改"待执行方案"，等这一个周期跑完（相位归零）才真正换过去。
 *
 *  显示部分在 CODE/LCD.c
 */

#define LED_PIN        PA8
#define LED_ON_LEVEL   GPIO_High        // 灯是低电平点亮的话，这两行换过来
#define LED_OFF_LEVEL  GPIO_Low

#define SCHEME_NUM     4
#define LED_PERIOD_MS  2000

// 每套方案各个相位的时长(ms)，偶数相位亮、奇数相位灭，每行加起来都是 2000
static const uint16_t phase_table[SCHEME_NUM][4] = {
    {1000, 1000,   0,    0},      // 1: 亮1000 灭1000
    {200,  1800,   0,    0},      // 2: 亮200  灭1800
    {200,  200,    200,  1400},   // 3: 亮200 灭200 亮200 灭1400
    {200,  200,    600,  1000},   // 4: 亮200 灭200 亮600 灭1000
};

// 每套方案有几个相位
static const uint8_t phase_num[SCHEME_NUM] = {2, 2, 4, 4};

volatile uint8_t g_led_cur_scheme  = 1;      // 当前正在跑的方案 1~4
volatile uint8_t g_led_pend_scheme = 1;      // 待执行方案，按键改这个
volatile uint8_t g_led_on          = 1;      // LED 状态 1亮 0灭

static uint8_t  phase_idx = 0;               // 走到第几个相位
static uint16_t phase_ms  = 0;               // 当前相位已经走了多少 ms
static uint32_t long_tick = 0;               // 长按计时用

void LED_Task_Init(void)
{
    NEUQ_GPIO_init(LED_PIN, GPIO_Mode_Output_PushPull, LED_OFF_LEVEL);

    phase_idx = 0;
    phase_ms  = 0;

    g_led_cur_scheme  = 1;
    g_led_pend_scheme = 1;
    g_led_on = 1;                            // 每套动作都是从亮开始

    NEUQ_GPIO_write(LED_PIN, LED_ON_LEVEL);
}

// 1ms 进来一次，在 TIM7 中断里
void LED_Task_1ms(void)
{
    uint8_t  sc = g_led_cur_scheme;
    uint16_t dur;

    if(sc < 1 || sc > SCHEME_NUM) sc = 1;

    dur = phase_table[sc - 1][phase_idx];
    if(dur == 0) dur = LED_PERIOD_MS;

    phase_ms++;
    if(phase_ms >= dur)
    {
        phase_ms = 0;
        phase_idx++;

        if(phase_idx >= phase_num[sc - 1])
        {
            // 一个完整周期跑完了，就在这个时刻换方案，这样周期长度不会被改掉
            phase_idx = 0;
            g_led_cur_scheme = g_led_pend_scheme;
        }

        g_led_on = (phase_idx % 2 == 0) ? 1 : 0;
        NEUQ_GPIO_write(LED_PIN, g_led_on ? LED_ON_LEVEL : LED_OFF_LEVEL);
    }
}

// 待执行方案加一或减一，超出 1~4 就绕回去
static void pend_step(int8_t dir)
{
    int16_t v = g_led_pend_scheme + dir;

    if(v > SCHEME_NUM) v = 1;
    if(v < 1)          v = SCHEME_NUM;

    g_led_pend_scheme = v;
}

static void key_handle(void)
{
    KEY_EVENT_ENUM ev;
    uint32_t now;

    // 取事件和清事件中间不能被中断插进来，不然可能丢事件
    __disable_irq();
    ev = NEUQ_Key_GetEvent();
    __enable_irq();

    if(ev == KEY_EVENT_NONE) return;

    // 长按的话每 10ms 就会来一次，这里限制成 600ms 才走一步
    if(ev == KEY_UP_LONG || ev == KEY_DOWN_LONG ||
       ev == KEY_FORWARD_LONG || ev == KEY_BACK_LONG)
    {
        now = HAL_GetTick();
        if(now - long_tick < 600) return;
        long_tick = now;
    }

    switch(ev)
    {
        case KEY_UP_SHORT:
        case KEY_UP_LONG:
        case KEY_FORWARD_SHORT:
        case KEY_FORWARD_LONG:
            pend_step(1);
            break;

        case KEY_DOWN_SHORT:
        case KEY_DOWN_LONG:
        case KEY_BACK_SHORT:
        case KEY_BACK_LONG:
            pend_step(-1);
            break;

        default:
            break;
    }
}

// **************************** 代码区域 ****************************
void User_Main(void){

    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();

    NEUQ_Key_Init();
    LCD_Menu_Init();

    // 要在开 1ms 定时器之前初始化
    LED_Task_Init();

    // 10ms 扫一次按键（扫两次才判定，相当于消抖）
    NEUQ_PIT_init_ms(PIT_CH1, KEY_SCAN_PERIOD_MS);
    // 1ms 跑一次 LED 时序
    NEUQ_PIT_init_ms(PIT_CH2, 1);

    while(1){

        key_handle();

        // ADC 电路坏了，电压先用 N/A 顶着
        LCD_Menu_Update(g_led_cur_scheme, g_led_pend_scheme, g_led_on, LCD_VOLTAGE_INVALID);
    }
}
