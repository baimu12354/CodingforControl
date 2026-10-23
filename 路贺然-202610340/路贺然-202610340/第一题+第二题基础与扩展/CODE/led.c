#include "NEUQ_lib_include.h"
#include "led.h"

// 初始化 LED 引脚，开始熄灭状态
void LED_Init(void)
{
    NEUQ_GPIO_init(LED_PIN, GPIO_Mode_Output_PushPull, LED_OFF_LEVEL);
}

// 点亮小灯
void LED_On(void)
{
    NEUQ_GPIO_write(LED_PIN, LED_ON_LEVEL);
}

// 熄灭小灯
void LED_Off(void)
{
    NEUQ_GPIO_write(LED_PIN, LED_OFF_LEVEL);
}

// 翻转小灯
void LED_Toggle(void)
{
    NEUQ_GPIO_toggle(LED_PIN);
}

// ==================== LED 闪烁方案=====================


typedef struct
{
    uint16_t dur_ms;   //持续时间
    uint8_t  level;    //亮灭
} Led;

// 四套方案
static const Led l1[] = { {1000,1}, {1000,0} };                        // 方案1：亮1000 灭1000
static const Led l2[] = { {200,1},  {1800,0} };                        // 方案2：亮200  灭1800
static const Led l3[] = { {200,1},  {200,0}, {200,1}, {1400,0} };      // 方案3：亮200 灭200 亮200 灭1400
static const Led l4[] = { {200,1},  {200,0}, {600,1}, {1000,0} };      // 方案4：亮200 灭200 亮600 灭1000

// 方案表：方案 = 名字 + 段列表
typedef struct
{
    const char    *name;   // 方案说明（ASCII，方便对照）
    const Led     *L;      // 段列表指针
    uint8_t        cnt;    // 段数
} Scheme;

static const Scheme scheme_table[4] = {
    [0] = {"1000on_1000off",             l1, 2},
    [1] = {"200on_1800off",              l2, 2},
    [2] = {"200on_200off_200on_1400off", l3, 4},
    [3] = {"200on_200off_600on_1000off", l4, 4},
};

static uint8_t             current_scheme    = 0;            // 当前正在执行的方案 0~3
static uint8_t             pending_scheme    = 0;            // 待执行方案 0~3
static uint16_t            elapsed_ms        = 0;            // 当前周期内已过 ms（0~1999
static volatile uint32_t   tick_10ms         = 0;            // 系统 10ms 单调计数

void LED_Scheme_Init(void)
{
    current_scheme = 0;
    pending_scheme = 0;
    elapsed_ms     = 0;
    LED_Off();
}

// 查表：方案在 t 毫秒时该亮(返回1)还是灭(返回0)
static uint8_t scheme_level_at(uint8_t scheme, uint16_t t)
{
    const Led *L  = scheme_table[scheme].L;
    uint8_t cnt   = scheme_table[scheme].cnt;
    uint16_t acc  = 0;

    for (uint8_t i = 0; i < cnt; i++)
    {
        acc += L[i].dur_ms;
        if (t < acc)
            return L[i].level;   // t 落在这一段
    }
    return 0;
}

// 每 10ms 由 PIT 中断调用一次。
// 按键只改 next measure，只有走到周期末尾才把它搬进 current_measrue，
void LED_Scheme_Tick10ms(void)
{
    tick_10ms++;

    // 1) 先按当前 elapsed_ms 查表输出（先输出，再推进）
    if (scheme_level_at(current_scheme, elapsed_ms))
        LED_On();
    else
        LED_Off();

    // 2) 推进 10ms；满 2000ms 进入下一周期
    elapsed_ms += 10;
    if (elapsed_ms >= 2000)
    {
        elapsed_ms = 0;
        current_scheme = pending_scheme;      // 待执行方案在下一周期起点生效
    }
}

void LED_Scheme_SetPending(uint8_t s)
{
    if (s < 4)
        pending_scheme = s;
}

uint8_t LED_Scheme_GetCurrent(void)
{
    return current_scheme;
}

uint8_t LED_Scheme_GetPending(void)
{
    return pending_scheme;
}

uint32_t LED_GetTick10ms(void)
{
    return tick_10ms;
}
