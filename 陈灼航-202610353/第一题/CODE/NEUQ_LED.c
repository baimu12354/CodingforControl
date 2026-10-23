#include "NEUQ_lib_include.h"

typedef struct{
    GPIO_Level level;    //该阶段输出的电平
    uint16_t   duration; //持续时间 ms
}LED_Step;

typedef struct{
    const LED_Step *steps; //指向阶段数组
    uint8_t         count; //这套方案一共有几个阶段
}LED_SchemeDef;

//方案1：亮1000ms -> 灭1000ms
static const LED_Step Scheme1[] = {
    {GPIO_High, 1000},  //第1阶段：亮 1000ms
    {GPIO_Low,  1000},  //第2阶段：灭 1000ms
};

//方案2：亮200ms -> 灭1800ms
static const LED_Step Scheme2[] = {
    {GPIO_High, 200},   //亮 200ms
    {GPIO_Low,  1800},  //灭 1800ms
};

//方案3：亮200 -> 灭200 -> 亮200 -> 灭1400
static const LED_Step Scheme3[] = {
    {GPIO_High, 200},   //亮 200ms
    {GPIO_Low,  200},   //灭 200ms
    {GPIO_High, 200},   //亮 200ms
    {GPIO_Low,  1400},  //灭 1400ms
};

//方案4：亮200 -> 灭200 -> 亮600 -> 灭1000
static const LED_Step Scheme4[] = {
    {GPIO_High, 200},   //亮 200ms
    {GPIO_Low,  200},   //灭 200ms
    {GPIO_High, 600},   //亮 600ms
    {GPIO_Low,  1000},  //灭 1000ms
};

static const LED_SchemeDef SchemeTable[LED_SCHEME_NUM] = {
    {Scheme1, 2},   //方案1：2 个阶段
    {Scheme2, 2},   //方案2：2 个阶段
    {Scheme3, 4},   //方案3：4 个阶段
    {Scheme4, 4},   //方案4：4 个阶段
};

static uint8_t  current_scheme; //当前正在执行的方案（1~4）
static uint8_t  pending_scheme; //待执行方案（1~4），当前周期结束后才生效
static uint8_t  step_index;     //当前方案内正在跑第几个阶段
static uint16_t step_elapsed;   //当前阶段已经跑了多少毫秒

// LED 初始化
void NEUQ_LED_Init(void){
    // 把 PA8 配成推挽输出，初始输出低电平
    NEUQ_GPIO_init(LED_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);

    // 状态复位：上电默认跑方案1
    current_scheme = LED_SCHEME_1;
    pending_scheme = LED_SCHEME_1;
    step_index     = 0; //从方案1的第0个阶段开始
    step_elapsed   = 0; //该阶段还没跑

    // 先输出方案1第一个阶段的电平
    NEUQ_GPIO_write(LED_PIN, SchemeTable[current_scheme-1].steps[0].level);
}

// 设置待执行方案
void NEUQ_LED_SetPending(uint8_t scheme){
    if(scheme >= LED_SCHEME_1 && scheme <= LED_SCHEME_NUM){
        pending_scheme = scheme;
    }
}

// 读当前正在执行的方案
uint8_t NEUQ_LED_GetCurrent(void){
    return current_scheme;
}

// 读待执行方案
uint8_t NEUQ_LED_GetPending(void){
    return pending_scheme;
}

// LED 状态机推进
void NEUQ_LED_Tick(void){
    // 先取出当前方案、当前阶段的定义
    const LED_SchemeDef *def  = &SchemeTable[current_scheme-1];
    const LED_Step      *step = &def->steps[step_index];

    step_elapsed += LED_TICK_MS; //本阶段累计时间 +10ms

    if(step_elapsed >= step->duration){
        step_elapsed -= step->duration; //把本阶段的时长减掉，多余时间留给下一阶段
        step_index++;                   //指向下一个阶段

        if(step_index >= def->count){
            step_index = 0;                        //回到这套方案的第一个阶段
            current_scheme = pending_scheme;       //方案在周期边界才切换
            def = &SchemeTable[current_scheme-1];  //重新查新方案的表
        }

        // 输出新阶段的电平
        NEUQ_GPIO_write(LED_PIN, def->steps[step_index].level);
    }
}
