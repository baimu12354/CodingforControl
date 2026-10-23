#include "led.h"
#include "config.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

//--------------------------- 4 套方案 ---------------------------
typedef struct{
    uint16_t time[4];           // 各段时长(ms)
    uint8_t  cnt;               // 段数
} Scheme_t;

static const Scheme_t schemes[SCHEME_COUNT] = {
    // 方案1：亮1000 → 灭1000
    {{1000, 1000},          2},
    // 方案2：亮200 → 灭1800
    {{200, 1800},           2},
    // 方案3：亮200 → 灭200 → 亮200 → 灭1400
    {{200, 200, 200, 1400}, 4},
    // 方案4：亮200 → 灭200 → 亮600 → 灭1000
    {{200, 200, 600, 1000}, 4},
};

//----------------------------------------------------------------
// 函数简介     设置 LED 电平
//----------------------------------------------------------------
void LED_Set(uint8_t on){
    NEUQ_GPIO_write(LED_PIN, on ? LED_ON : LED_OFF);
}

//----------------------------------------------------------------
// 函数简介     计算某方案在周期内 position_ms 时刻的 LED 电平
// 参数说明     scheme       方案编号 1~4
// 参数说明     position_ms  周期内位置，0~CYCLE_MS-1
// 返回参数     0 灭，1 亮
//----------------------------------------------------------------
uint8_t LED_GetLevel(uint8_t scheme, uint32_t position_ms){
    const Scheme_t *s = &schemes[scheme - 1];
    for (uint8_t i = 0; i < s->cnt; i++){
        if (position_ms < s->time[i]) return (i % 2) == 0;   // 偶数段亮，奇数段灭
        position_ms -= s->time[i];
    }
    return 0;
}
