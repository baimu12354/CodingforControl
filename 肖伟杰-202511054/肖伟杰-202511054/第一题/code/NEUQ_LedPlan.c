/*********************************************************************************************************************
* 文件名称          NEUQ_LedPlan
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第一题 LED 时序控制：四套方案，每套完整周期 2 s
*
* 方案表（一个完整周期内的动作，合计均为 2000 ms）
*   1   亮 1000 → 灭 1000
*   2   亮  200 → 灭 1800
*   3   亮  200 → 灭  200 → 亮  200 → 灭 1400
*   4   亮  200 → 灭  200 → 亮  600 → 灭 1000
*
* 关键设计（对应任务书第(2)条：不得暂停、拉长或反复重置当前动作）
*   ① LED 电平只由"当前时刻 - 本周期起点"和"当前方案"查表得出，没有任何 Delay；
*   ② 按键只写 g_led_plan_pending，绝不碰周期起点，也不碰 LED 引脚；
*   ③ 周期起点用 += 2000 固定步长累加（而不是取 now），所以周期严格 2 s、永不漂移；
*   ④ 本函数在 1 ms PIT 中断里调用，主循环刷 LCD 或连续按键都不会影响 LED 时序。
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "NEUQ_LedPlan.h"

/* ------------------------- 方案动作表 ------------------------- */
/* level：1=亮 0=灭；ms：该段持续时间；以 ms==0 作为结束标记 */
typedef struct
{
    uint8_t  level;
    uint16_t ms;
} LedSegType;

static const LedSegType plan_seg1[] = { {1, 1000}, {0, 1000}, {0, 0} };                             /* 方案 1 */
static const LedSegType plan_seg2[] = { {1,  200}, {0, 1800}, {0, 0} };                             /* 方案 2 */
static const LedSegType plan_seg3[] = { {1,  200}, {0,  200}, {1,  200}, {0, 1400}, {0, 0} };       /* 方案 3 */
static const LedSegType plan_seg4[] = { {1,  200}, {0,  200}, {1,  600}, {0, 1000}, {0, 0} };       /* 方案 4 */

static const LedSegType *const plan_table[LED_PLAN_NUM] =
{
    plan_seg1, 
    plan_seg2, 
    plan_seg3, 
    plan_seg4
};

/* ------------------------- 内部状态 ------------------------- */
volatile int32_t g_led_plan_active  = 1;    /* 当前方案 */
volatile int32_t g_led_plan_pending = 1;    /* 待执行方案 */

static uint32_t s_cycle_start = 0;          /* 本周期起点的时间戳(ms) */
static uint8_t  s_led_level   = 0xFF;       /* 0xFF 表示"未知"，用于强制第一次写引脚 */

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     把方案序号(1~4)规整到合法范围
*------------------------------------------------------------------------------------------------------------------*/
static uint8_t LedPlan_ClampPlan(int32_t plan)
{
    if (plan < 1)                     return 1;
    if (plan > (int32_t)LED_PLAN_NUM) return (uint8_t)LED_PLAN_NUM;
    return (uint8_t)plan;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     求某方案一个周期的总时长(ms)（四个方案都应该是 2000）
*------------------------------------------------------------------------------------------------------------------*/
uint16_t NEUQ_LedPlan_TotalMs(uint8_t plan)
{
    const LedSegType *segs = plan_table[LedPlan_ClampPlan(plan) - 1];
    uint32_t total = 0;

    for (uint8_t i = 0; segs[i].ms != 0; i++)
    {
        total += segs[i].ms;
    }
    return (uint16_t)total;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     根据"周期内已过去的时间"查表得到应有的 LED 电平
*------------------------------------------------------------------------------------------------------------------*/
static uint8_t LedPlan_LevelAt(uint8_t plan, uint32_t elapsed_ms)
{
    const LedSegType *segs = plan_table[LedPlan_ClampPlan(plan) - 1];
    uint32_t acc = 0;

    for (uint8_t i = 0; segs[i].ms != 0; i++)
    {
        acc += segs[i].ms;
        if (elapsed_ms < acc)
        {
            return segs[i].level;      /* 落在这一段里 */
        }
    }
    return 0;                          /* 理论上不会走到这里：超出周期视为熄灭 */
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     LED 时序初始化（上电即从方案 1 开始循环）
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_LedPlan_Init(void)
{
    NEUQ_GPIO_init(LED_PLAN_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);

    g_led_plan_active  = 1;            /* 上电从方案 1 开始运行 */
    g_led_plan_pending = 1;

    s_cycle_start = HAL_GetTick();     /* 本周期起点 */
    s_led_level   = 0;
    NEUQ_GPIO_write(LED_PLAN_PIN, GPIO_Low);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     时序引擎，每 1 ms 调用一次（放在 PIT 中断回调中）
* 参数说明     now_ms：当前毫秒时间戳(HAL_GetTick())
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_LedPlan_Tick(uint32_t now_ms)
{
    uint8_t level;

    /* ① 周期边界处理：把"待执行方案"提交为"当前方案"，在下一周期起点生效 */
    while ((uint32_t)(now_ms - s_cycle_start) >= LED_PLAN_PERIOD_MS)
    {
        s_cycle_start += LED_PLAN_PERIOD_MS;          /* ★固定步长累加 → 周期严格 2 s，不漂移 */
        g_led_plan_active = (int32_t)LedPlan_ClampPlan(g_led_plan_pending);   /* ★只在周期起点提交 */
    }

    /* ② 依据"当前时刻 - 周期起点"和"当前方案"决定电平；只有变化时才写引脚 */
    level = LedPlan_LevelAt((uint8_t)g_led_plan_active, (uint32_t)(now_ms - s_cycle_start));
    if (level != s_led_level)
    {
        s_led_level = level;
        NEUQ_GPIO_write(LED_PLAN_PIN, level ? GPIO_High : GPIO_Low);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介     读取当前方案 / 待执行方案 / 本周期已过时间
*------------------------------------------------------------------------------------------------------------------*/
uint8_t NEUQ_LedPlan_GetActive(void)
{
    return LedPlan_ClampPlan(g_led_plan_active);
}

uint8_t NEUQ_LedPlan_GetPending(void)
{
    return LedPlan_ClampPlan(g_led_plan_pending);
}

uint32_t NEUQ_LedPlan_ElapsedMs(void)
{
    return (uint32_t)(HAL_GetTick() - s_cycle_start);
}
