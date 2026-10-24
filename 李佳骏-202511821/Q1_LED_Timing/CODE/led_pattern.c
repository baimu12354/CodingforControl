/* led_pattern.c - 四套 LED 时序 */

#include "led_pattern.h"
#include "board_config.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

/* 一个动作段：亮或灭持续多少毫秒 */
typedef struct {
    uint16_t duration_ms;
    uint8_t  on;            /* 1 = 亮，0 = 灭 */
} led_seg_t;

/* 四套方案的段表。每套各段之和都严格等于 2000 ms。 */
static const led_seg_t s_pattern_1[] = { { 1000, 1 }, { 1000, 0 } };
static const led_seg_t s_pattern_2[] = { {  200, 1 }, { 1800, 0 } };
static const led_seg_t s_pattern_3[] = { {  200, 1 }, {  200, 0 }, { 200, 1 }, { 1400, 0 } };
static const led_seg_t s_pattern_4[] = { {  200, 1 }, {  200, 0 }, { 600, 1 }, { 1000, 0 } };

static const led_seg_t *const s_pattern_tab[T1_PATTERN_COUNT] = {
    s_pattern_1, s_pattern_2, s_pattern_3, s_pattern_4
};

static const uint8_t s_pattern_len[T1_PATTERN_COUNT] = { 2, 2, 4, 4 };

static const char *const s_pattern_desc[T1_PATTERN_COUNT] = {
    "1000/1000",
    "200/1800",
    "200/200/200/1400",
    "200/200/600/1000",
};

/* 点亮和熄灭对应的实际电平，由 T1_LED_ON_LEVEL 决定 */
#define LED_LEVEL_ON    (T1_LED_ON_LEVEL)
#define LED_LEVEL_OFF   ((T1_LED_ON_LEVEL) == GPIO_High ? GPIO_Low : GPIO_High)

static uint8_t  s_current = 0;          /* 正在执行的方案号 */
static uint8_t  s_pending = 0;          /* 待执行方案号 */
static uint32_t s_elapsed_ms = 0;       /* 当前周期内已过毫秒数 */
static volatile uint8_t s_ui_dirty = 1; /* 界面是否需要重画（中断里也会置位） */

/* 按当前周期偏移查段表，决定亮灭 */
static void LedPattern_ApplyOutput(void)
{
    const led_seg_t *seg = s_pattern_tab[s_current];
    uint8_t  len   = s_pattern_len[s_current];
    uint32_t acc   = 0;
    uint8_t  on    = 0;
    uint8_t  i;

    for (i = 0; i < len; i++)
    {
        acc += seg[i].duration_ms;
        if (s_elapsed_ms < acc)
        {
            on = seg[i].on;
            break;
        }
    }

    NEUQ_GPIO_write(T1_LED_PIN, on ? LED_LEVEL_ON : LED_LEVEL_OFF);
}

void LedPattern_Init(void)
{
    NEUQ_GPIO_init(T1_LED_PIN, GPIO_Mode_Output_PushPull, LED_LEVEL_OFF);

    s_current     = 0;
    s_pending     = 0;
    s_elapsed_ms  = 0;
    s_ui_dirty    = 1;

    LedPattern_ApplyOutput();
}

/* PIT 中断调用：推进时序并写 LED，避免刷屏影响边沿 */
void LedPattern_TickIsr(void)
{
    s_elapsed_ms += T1_TICK_MS;

    /* 周期边界：走到这里才允许换方案 */
    if (s_elapsed_ms >= T1_CYCLE_MS)
    {
        s_elapsed_ms -= T1_CYCLE_MS;
        if (s_pending != s_current)
        {
            s_current  = s_pending;
            s_ui_dirty = 1;             /* 真正切换了，界面要更新 */
        }
    }

    LedPattern_ApplyOutput();
}

void LedPattern_SelectNext(void)
{
    s_pending = (uint8_t)((s_pending + 1U) % T1_PATTERN_COUNT);
    s_ui_dirty = 1;
}

void LedPattern_SelectPrev(void)
{
    s_pending = (uint8_t)((s_pending + T1_PATTERN_COUNT - 1U) % T1_PATTERN_COUNT);
    s_ui_dirty = 1;
}

uint8_t LedPattern_GetCurrent(void)
{
    return s_current;
}

uint8_t LedPattern_GetPending(void)
{
    return s_pending;
}

uint8_t LedPattern_TakeUiDirty(void)
{
    uint8_t dirty = s_ui_dirty;
    s_ui_dirty = 0;
    return dirty;
}

const char *LedPattern_GetSummary(uint8_t idx)
{
    if (idx >= T1_PATTERN_COUNT)
    {
        idx = 0;
    }
    return s_pattern_desc[idx];
}
