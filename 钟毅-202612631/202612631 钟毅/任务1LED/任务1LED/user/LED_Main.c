/*********************************************************************************************************************
* NEUQ Library (东北大学秦皇岛分校 开源库)
* Copyright (c) 2025 NEUQ SmartCar Lab
* ... (保留你文件开头的版权声明) ...
* 文件名称          NEUQ_Main
* 修改记录          2025-09-05 张瑞 first version
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

/* ---------- 1. 引脚与硬件参数：改硬件只改这里 ---------- */
#define PIN_LED      PA8                 // 板载 LED
#define PIN_KEY      PA0                 // 板载按键 K0（上拉输入，按下为低电平）
#define ADC_CH       ADC1_CH3_PA3        // 考核要求的电压采集引脚 PA3
#define DIVIDER_RATIO 1.0f               // 分压比。直接采<=3.3V信号保持1.0

/* ---------- 2. 四套动作表（剧本） ----------
 * 每一行代表一套动作，每一对{持续毫秒, 是否点亮}
 * 填 0 表示结束 */
typedef struct { uint16_t ms; uint8_t on; } Step;
static const Step SCHEME[4][5] = {
    { {1000,1}, {1000,0}, {0,0}, {0,0}, {0,0} },              // 方案1
    { { 200,1}, {1800,0}, {0,0}, {0,0}, {0,0} },              // 方案2
    { { 200,1}, { 200,0}, {200,1}, {1400,0}, {0,0} },         // 方案3
    { { 200,1}, { 200,0}, {600,1}, {1000,0}, {0,0} },         // 方案4
};

/* ---------- 3. 全局变量（状态机） ---------- */
static volatile uint32_t tick_ms       = 0;   // 开机以来的毫秒数，中断里每 1ms 加 1
static volatile uint8_t  pending_scheme = 0;   // 按键选中的"待执行"方案(0~3)
static          uint8_t  cur_scheme    = 0;    // 正在执行的方案(0~3)
static          uint8_t  step_idx      = 0;    // 当前走到本方案的第几步
static          uint32_t step_start    = 0;    // 当前这一步开始时的 tick_ms
static          uint8_t  key_cnt       = 0;    // 按键消抖计数
static          uint32_t last_key_scan = 0;    // 上次扫描按键的时刻
static          uint32_t last_adc      = 0;    // 上次采电压的时刻
static          uint32_t last_draw     = 0;    // 上次刷屏幕的时刻
static          float    volt_show     = 0.0f; // 换算好的电池电压

/* ---------- 4. LED 时序引擎（被1ms中断调用） ---------- */
void LED_Tick_1ms(void)
{
    tick_ms++;
    // 判断当前这一步时间到了没
    if (tick_ms - step_start >= SCHEME[cur_scheme][step_idx].ms) {
        step_idx++; // 走下一步
        if (SCHEME[cur_scheme][step_idx].ms == 0) { // 跑完一个完整周期
            step_idx    = 0; // 回到第0步
            cur_scheme  = pending_scheme; // ★周期边界切换方案
        }
        step_start = tick_ms; // 记录新起点
        NEUQ_GPIO_write(PIN_LED, SCHEME[cur_scheme][step_idx].on ? GPIO_High : GPIO_Low);
    }
}

/* ---------- 5. 主函数 ---------- */
void User_Main(void)
{
    // 初始化
    NEUQ_Debug_init();
    NEUQ_Delay_init();
    NEUQ_LCD200_init(); // 屏幕初始化
    NEUQ_GPIO_init(PIN_LED, GPIO_Mode_Output_PushPull, GPIO_High);  // LED
    NEUQ_GPIO_init(PIN_KEY, GPIO_Mode_Input_PullUp,  GPIO_High);    // 按键
    adc_init(ADC_CH, ADC_12BIT); // PA3 ADC初始化
    NEUQ_GPIO_write(PIN_LED, SCHEME[0][0].on ? GPIO_High : GPIO_Low); // 点亮第1步
    NEUQ_PIT_init_ms(PIT_CH1, 1); // ★开启1ms定时器中断

    while (1)
    {
        // ① 每10ms扫一次按键（非阻塞消抖）
        if (tick_ms - last_key_scan >= 10) {
            last_key_scan = tick_ms;
            if (NEUQ_GPIO_read(PIN_KEY) == GPIO_Low) {
                key_cnt++;
            } else {
                if (key_cnt >= 2 && key_cnt < 50) {
                    pending_scheme = (pending_scheme + 1) % 4; // 短按方案+1
                }
                key_cnt = 0;
            }
        }

        // ② 每100ms采一次电压
        if (tick_ms - last_adc >= 100) {
            last_adc = tick_ms;
            uint16_t raw = adc_mean_filter_convert(ADC_CH, 8);
            volt_show = (float)raw / 4095.0f * 3.3f * DIVIDER_RATIO;
        }

        // ③ 每200ms刷一次屏幕
        if (tick_ms - last_draw >= 200) {
            last_draw = tick_ms;
            NEUQ_LCD200_DisplayString(  0,  0, "Cur :");
            NEUQ_LCD200_Display_int32( 40,  0, (int32_t)cur_scheme + 1, 1);
            NEUQ_LCD200_DisplayString( 70,  0, "Next:");
            NEUQ_LCD200_Display_int32(110,  0, (int32_t)pending_scheme + 1, 1);
            NEUQ_LCD200_DisplayString(  0, 20, "Volt:");
            NEUQ_LCD200_DisplayFloat( 40, 20, volt_show, 5, 2);
            NEUQ_LCD200_DisplayString( 90, 20, "V");
        }
    }
}