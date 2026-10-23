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
* 2026-09-29        用户        适配 NEUQ_lib API，实现 LED 时序控制功能
* 2026-10-10        用户        重构：LED 驱动抽成函数、修 LCD 显示慢一拍、循环计数与中断解耦
* 2026-10-18        用户        答辩精简：按键改按下沿检测消抖，去掉状态机与指针
********************************************************************************************************************/

#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Main.h"

// **************************** 功能说明 ****************************
// 4 个按键分别选择 4 套 LED(PA8) 闪烁方案，LCD 显示当前/待执行方案。
// 接线：PE9→方案1  PE8→方案2  PE7→方案3  PE10→方案4（低电平按下，上拉输入）
//        PA8 板载 LED。
// 原理：主循环每 20ms 走一拍，100 拍 = 2s 一个周期；按键按下沿改 next_scheme，
//       在下一周期起点生效（不打断当前周期）。
// **************************** 功能说明 ****************************

// ===== 全局变量（NEUQ_Interrupts.c 有 extern 声明，保留 volatile）=====
volatile uint8_t  current_scheme = 1;   // 当前正在执行的方案
volatile uint8_t  next_scheme    = 1;   // 按键选好、待生效的方案
volatile uint16_t time_tick;            // PIT 中断累计刻度（本程序用轮询计时，未使用）

// ===== 按键引脚 =====
#define KEY_UP      PE9    // 方案 1
#define KEY_DOWN    PE8    // 方案 2
#define KEY_FORWARD PE7    // 方案 3
#define KEY_BACK    PE10   // 方案 4

// 每个按键上一次的电平，用于检测“按下沿”：本次低 + 上次高 = 新按下一次
uint8_t prev_up      = 1;
uint8_t prev_down    = 1;
uint8_t prev_forward = 1;
uint8_t prev_back    = 1;

// 扫描 4 个按键；每 20ms 调一次，采样间隔本身即可完成消抖
void Key_Scan(void)
{
    uint8_t cur;

    cur = (NEUQ_GPIO_read(KEY_UP) == GPIO_Low);      // 本次是否按下
    if (cur && !prev_up)      next_scheme = 1;        // 按下沿 → 方案 1
    prev_up = cur;

    cur = (NEUQ_GPIO_read(KEY_DOWN) == GPIO_Low);
    if (cur && !prev_down)    next_scheme = 2;        // 按下沿 → 方案 2
    prev_down = cur;

    cur = (NEUQ_GPIO_read(KEY_FORWARD) == GPIO_Low);
    if (cur && !prev_forward) next_scheme = 3;        // 按下沿 → 方案 3
    prev_forward = cur;

    cur = (NEUQ_GPIO_read(KEY_BACK) == GPIO_Low);
    if (cur && !prev_back)    next_scheme = 4;        // 按下沿 → 方案 4
    prev_back = cur;
}

// 按方案 scheme 和节拍 tick（0~99，每拍 20ms）驱动 PA8 的 LED
void LED_Update(uint8_t scheme, uint16_t tick)
{
    switch (scheme)
    {
        case 1:   // 1s 亮、1s 灭
            NEUQ_GPIO_write(PA8, (tick < 50) ? GPIO_High : GPIO_Low);
            break;
        case 2:   // 0.2s 亮、1.8s 灭
            NEUQ_GPIO_write(PA8, (tick < 10) ? GPIO_High : GPIO_Low);
            break;
        case 3:   // 短亮-灭-短亮-灭
            NEUQ_GPIO_write(PA8, (tick < 10 || (tick >= 20 && tick < 30)) ? GPIO_High : GPIO_Low);
            break;
        case 4:   // 短亮-灭-长亮-灭
            NEUQ_GPIO_write(PA8, (tick < 10 || (tick >= 20 && tick < 50)) ? GPIO_High : GPIO_Low);
            break;
        default:  // 方案越界则熄灭
            NEUQ_GPIO_write(PA8, GPIO_Low);
            break;
    }
}

// 主循环入口
void User_Main(void)
{
    // ----- 硬件初始化 -----
    NEUQ_GPIO_init(PA8,         GPIO_Mode_Output_PushPull, GPIO_Low);   // LED
    NEUQ_GPIO_init(KEY_UP,      GPIO_Mode_Input_PullUp, GPIO_High);     // 4 按键上拉输入
    NEUQ_GPIO_init(KEY_DOWN,    GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_FORWARD, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_BACK,    GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_Delay_init();                                    // 延时（NEUQ_Delay_ms 依赖）
    NEUQ_Uart_init(UART_0, UART_BAUD_115200);             // LCD 的 printf 重定向，需先初始化
    NEUQ_LCD200_init();                                   // 初始化 LCD

    // 文字标签静态不变，只画一次
    NEUQ_LCD200_DisplayString(0, 0, "Current:");
    NEUQ_LCD200_DisplayString(0, 30, "Next:");

    while (1)
    {
        uint16_t tick;                 // 局部节拍计数

        current_scheme = next_scheme;  // 周期起点：应用按键选好的方案

        for (tick = 0; tick < 100; tick++)   // 100 拍 × 20ms = 2s
        {
            Key_Scan();                       // 扫描按键，可能改 next_scheme
            LED_Update(current_scheme, tick); // 按当前方案驱动 LED
            NEUQ_Delay_ms(20);
        }

        // 周期结束：刷新两个数字位（Current=本周期运行方案，Next=待执行方案）
        NEUQ_LCD200_Display_int32(192, 0, current_scheme, 1);
        NEUQ_LCD200_Display_int32(120, 30, next_scheme, 1);
    }
}
