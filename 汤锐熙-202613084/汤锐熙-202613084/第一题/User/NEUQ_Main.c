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
#include "NEUQ_Display_Refresh.h"

// **************************** 代码区域 ****************************
void User_Main(void){
    // 按键消抖状态（主循环轮询用）
    const GPIO_Ports_ENUM btn_pin[4] = {BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN};
    uint8_t  was_low[4]      = {0};   // 上一时刻是否为低（用于检测下降沿）
    uint8_t  armed[4]        = {0};   // 本次按下是否已算过
    uint32_t stable_start[4] = {0};   // 本次按下起始时间戳(ms)

    // 初始化默认调试串口（LCD 内部用 printf，必须先初始化）
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();

    // LED 输出，高电平点亮
    NEUQ_GPIO_init(LED_PIN, GPIO_Mode_Output_PushPull, GPIO_High);

    // 四个按键：输入上拉（按下 = 低电平）
    NEUQ_GPIO_init(BTN1_PIN, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(BTN2_PIN, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(BTN3_PIN, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(BTN4_PIN, GPIO_Mode_Input_PullUp, GPIO_High);

    // 屏幕
    NEUQ_LCD200_init();
    NEUQ_Display_Refresh_Init();

    // 启动 1ms 定时中断（只负责 LED 时序与 2s 周期边界）
    NEUQ_PIT_init_ms(PIT_CH1, 1);

    // 主循环：轮询按键 + 消抖；需要时刷新屏幕
    while(1){
        uint32_t now = HAL_GetTick();   // 系统毫秒时间戳

        // ① 按键轮询 + 消抖（非阻塞，用真实毫秒计时，不占用中断）
        for(uint8_t i = 0; i < 4; i++){
            if(NEUQ_GPIO_read(btn_pin[i]) == GPIO_Low){
                if(was_low[i] == 0){                 // 下降沿：刚按下
                    was_low[i] = 1;
                    stable_start[i] = now;
                }
                if((now - stable_start[i] >= DEBOUNCE_MS) && (armed[i] == 0)){
                    armed[i]    = 1;
                    g_led_next  = (uint8_t)(i + 1);  // 键1→方案1，键2→方案2 ...
                    g_led_dirty = 1;
                }
            }else{
                was_low[i] = 0;   // 松开，允许下一次按下
                armed[i]   = 0;
            }
        }

        // ② 屏幕刷新
        if(g_led_dirty){
            g_led_dirty = 0;
            NEUQ_Display_Refresh_Update();
        }
    }
}
