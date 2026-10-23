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
* 文件名称          NEUQ_LED
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2026-10-01        ——                  四套 LED 动作（周期 2 秒）
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "NEUQ_LED.h"

/* ==================== LED 引脚配置（请按实际接线修改） ==================== */
#define LED1_PIN   PD8     // LED1，对应 light1
#define LED2_PIN   PD10    // LED2，对应 light2
#define LED3_PIN   PD12    // LED3，对应 light3
#define LED4_PIN   PD14    // LED4，对应 light4

#define LED_NUM    4       // LED 数量
#define PERIOD_MS  2000    // 每个动作的周期：2 秒

static const GPIO_Ports_ENUM g_led_pins[LED_NUM] = {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};

/* ==================== LED 底层驱动 ==================== */

// 点亮 / 熄灭单个 LED（idx：0=LED1 ... 3=LED4）
static void LED_SetOne(uint8_t idx, uint8_t on){
    NEUQ_GPIO_write(g_led_pins[idx], on ? GPIO_High : GPIO_Low);
}

// 熄灭全部 LED（每个方案只保留一个灯时，先把其余灯熄灭）
static void LED_AllOff(void){
    for(uint8_t i = 0; i < LED_NUM; i++){
        LED_SetOne(i, 0);
    }
}

// LED 引脚初始化（在 User_Main 中调用）
void LED_Init(void){
    for(uint8_t i = 0; i < LED_NUM; i++){
        NEUQ_GPIO_init(g_led_pins[i], GPIO_Mode_Output_PushPull, GPIO_Low);
    }
}

/* ==================== 各方案的亮灭状态（周期均为 2 秒） ==================== */
/*
 * 写法说明：
 *   HAL_GetTick() 返回系统上电以来的毫秒数，% PERIOD_MS 后得到当前时刻在
 *   2 秒周期内的位置 t（0 ~ 1999），再按各方案的时间段判断亮(1)或灭(0)。
 */

// light1：亮 1.0 秒 → 灭 1.0 秒
static uint8_t State_Light1(void){
    uint32_t t = HAL_GetTick() % PERIOD_MS;
    return t < 1000;
}

// light2：亮 0.2 秒 → 灭 1.8 秒
static uint8_t State_Light2(void){
    uint32_t t = HAL_GetTick() % PERIOD_MS;
    return t < 200;
}

// light3：亮 0.2 秒 → 灭 0.2 秒 → 亮 0.2 秒 → 灭 1.4 秒
static uint8_t State_Light3(void){
    uint32_t t = HAL_GetTick() % PERIOD_MS;
    return (t < 200) || (t >= 400 && t < 600);
}

// light4：亮 0.2 秒 → 灭 0.2 秒 → 亮 0.6 秒 → 灭 1.0 秒
static uint8_t State_Light4(void){
    uint32_t t = HAL_GetTick() % PERIOD_MS;
    return (t < 200) || (t >= 400 && t < 1000);
}

/* ==================== 四套 LED 动作 ==================== */
/*
 * 【连接点⑥ 实现】LED 程序与菜单程序的交汇处（函数体）
 *   下面的 4 个函数在 NEUQ_LED.h 中声明（见①），
 *   由菜单 NEUQ_Key.c 绑定并调用（见③④⑤）。
 *   按下确认键后，Menu_Task() 通过函数指针 g_current->scheme() 调用到这里执行。
 *   每个方案只点亮一个对应的 LED，其余 LED 熄灭。
 */

// 方案1：仅 LED1 按 light1 节奏闪
void LED_Scheme_Light1(void){
    LED_AllOff();
    LED_SetOne(0, State_Light1());
}

// 方案2：仅 LED2 按 light2 节奏闪
void LED_Scheme_Light2(void){
    LED_AllOff();
    LED_SetOne(1, State_Light2());
}

// 方案3：仅 LED3 按 light3 节奏闪
void LED_Scheme_Light3(void){
    LED_AllOff();
    LED_SetOne(2, State_Light3());
}

// 方案4：仅 LED4 按 light4 节奏闪
void LED_Scheme_Light4(void){
    LED_AllOff();
    LED_SetOne(3, State_Light4());
}
