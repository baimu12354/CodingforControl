#include "code.h"
#include "led.h"
#include "key.h"
#include "display.h"
#include "config.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

// 毫秒时基：由 NEUQ_Interrupts.c 中 NEUQ_PIT_1_CALLBACK 每 1ms 自增
volatile uint32_t Tick_ms = 0;

//--------------------------- 运行状态 ---------------------------
static uint8_t  current    = 1;    // 当前正在执行的方案
static uint8_t  next       = 1;    // 待执行方案
static uint32_t last_cycle = 0;    // 上次所在周期编号

//----------------------------------------------------------------
// 函数简介     初始化 LED / 按键 / LCD / PIT
//----------------------------------------------------------------
void Code_Init(void){
    // LED
    NEUQ_GPIO_init(LED_PIN, GPIO_Mode_Output_PushPull, LED_OFF);
    // 按键（低电平有效）
    NEUQ_GPIO_init(KEY1_PIN, GPIO_Mode_Input_PullUp, GPIO_Low);
    NEUQ_GPIO_init(KEY2_PIN, GPIO_Mode_Input_PullUp, GPIO_Low);
    NEUQ_GPIO_init(KEY3_PIN, GPIO_Mode_Input_PullUp, GPIO_Low);
    NEUQ_GPIO_init(KEY4_PIN, GPIO_Mode_Input_PullUp, GPIO_Low);
    // LCD
    NEUQ_LCD200_init();
    // 1ms 时基
    NEUQ_PIT_init_ms(PIT_CH1, 1);
}

//----------------------------------------------------------------
// 函数简介     主循环任务：LED 时序 + 按键切换 + LCD 刷新
//----------------------------------------------------------------
void Code_Task(void){
    // 1) 读取当前绝对时间
    uint32_t now      = Tick_ms;
    uint32_t cycle    = now / CYCLE_MS;    // 当前处于第几个周期
    uint32_t position = now % CYCLE_MS;    // 当前处于周期中的位置 0~1999

    // 2) 周期边界：把待执行方案提升为当前方案
    if (cycle != last_cycle){
        last_cycle = cycle;
        current    = next;
    }

    // 3) 设置 LED 电平
    LED_Set(LED_GetLevel(current, position));

    // 4) 按键扫描
    Key_Scan();
    uint8_t e = Key_GetEvent();
    if (e) next = e;

    // 5) LCD 刷新
    Display_Update(current, next);
}
