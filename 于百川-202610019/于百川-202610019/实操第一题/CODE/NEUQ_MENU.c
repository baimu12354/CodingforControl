#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_MENU.h"
#include "NEUQ_KEY.h"       /* 按键模块：NEUQ_KEY_Init / KEY_Scan / NEUQ_KEY_Get */

/* ============================ 1. 方案段表：每套完整周期 = 2000ms ============================ */
typedef struct {
    uint8_t  on;      /* 1=亮  0=灭 */
    uint16_t ms;      /* 本段时长(ms) */
} Step_t;

/* 题目给定的四套方案（一个完整周期均为 2s） */
static const Step_t s1[] = {{1, 1000}, {0, 1000}};
static const Step_t s2[] = {{1, 200},  {0, 1800}};
static const Step_t s3[] = {{1, 200}, {0, 200}, {1, 200}, {0, 1400}};
static const Step_t s4[] = {{1, 200}, {0, 200}, {1, 600}, {0, 1000}};

static const Step_t *const steps[4] = {s1, s2, s3, s4};
static const uint8_t  step_num[4]   = {2, 2, 4, 4};

/* ============================ 2. LED 状态机（PIT1 = TIM6 1ms 中断里运行） ============================ */
static volatile uint8_t  cur_scheme    = 1;   /* 当前运行方案 */
static volatile uint8_t  pend_scheme   = 1;   /* 待执行方案（按键只改它） */
static volatile uint8_t  scheme_changed = 0;  /* 周期切换标志：中断置1，主循环消费后清0（用于同步刷新 Current 行） */
static volatile uint8_t  cur_step      = 0;   /* 当前段序号 */
static volatile uint16_t step_tick     = 0;   /* 本段已走 ms */

void LED_Tick(void)
{
    const Step_t *p = steps[cur_scheme - 1];

    if (++step_tick >= p[cur_step].ms) {        /* 本段计时结束 */
        step_tick = 0;
        cur_step++;
        if (cur_step >= step_num[cur_scheme - 1]) {
            /* 完整 2s 周期结束 -> 下一周期起点：待执行方案生效 */
            cur_step        = 0;
            cur_scheme      = pend_scheme;
            scheme_changed  = 1;    /* 通知主循环刷新菜单，让 Current 行与实际运行方案同步 */
        }
        /* 应用新段电平（新周期的首段也在这里生效，四套方案首段均为“亮”） */
        NEUQ_GPIO_write(LED_PIN,
            steps[cur_scheme - 1][cur_step].on ? GPIO_High : GPIO_Low);
    }
}

/* ============================ 3. 单页显示（LCD200 横屏 320x240，ASCII_Font24） ============================ */
#define MENU_TITLE_Y   8
#define MENU_CUR_Y     40
#define MENU_PEND_Y    80
#define MENU_ADC_Y     120

/* 擦除整行再重画，避免整屏清屏闪烁 */
static void menu_draw_line(uint16_t y, const char *str)
{
    NEUQ_LCD200_ClearRect(0, y, 320, 26);
    NEUQ_LCD200_DisplayString(0, y, (char *)str);
}

/* 重画“当前方案 / 待执行方案”两行（按键变化或周期切换后调用） */
static void menu_refresh_scheme(void)
{
    char buf[20];

    sprintf(buf, "Current: %d", (int)cur_scheme);
    menu_draw_line(MENU_CUR_Y, buf);

    sprintf(buf, "Pending: %d", (int)pend_scheme);
    menu_draw_line(MENU_PEND_Y, buf);
}

/* 刷新 PA3 电压显示：仅在该值变化时重画，减小 SPI 刷新量、避免闪烁 */
static void menu_refresh_adc(void)
{
    static uint16_t last_mv = 0xFFFF;
    uint16_t adc_val;
    uint16_t mv;
    char buf[20];

    adc_val = adc_mean_filter_convert(ADC1_CH3_PA3, 5);            /* 5 次均值滤波 */
    mv      = (uint16_t)(((uint32_t)adc_val * 3300UL) / 4095UL);   /* 12bit -> mV */

    if (mv == last_mv) return;
    last_mv = mv;

    sprintf(buf, "PA3: %d.%02dV", mv / 1000, (mv % 1000) / 10);
    menu_draw_line(MENU_ADC_Y, buf);
}

/* ============================ 4. 初始化 ============================ */
void LED_Menu_Init(void)
{
    /* LED：PA8 推挽输出，初始低（状态机首段会点亮） */
    NEUQ_GPIO_init(LED_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
    /* 按键：PE9/PE8 上拉输入，由按键模块统一初始化 */
    NEUQ_KEY_Init();
    /* PA3：ADC1 通道3，12 位（验收：改变 PA3 输入电压，菜单电压随之变化） */
    adc_init(ADC1_CH3_PA3, ADC_12BIT);

    /* LCD：横屏，白底黑字，ASCII 24 点阵 */
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font24);
    NEUQ_LCD200_ShowNumMode(Fill_Space);
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(0, MENU_TITLE_Y, "LED TIMING CONTROL");

    /* 方案1 首段：亮（上电即开始循环） */
    NEUQ_GPIO_write(LED_PIN, GPIO_High);

    /* 画菜单初值 */
    menu_refresh_scheme();
    menu_refresh_adc();

    /* PIT1 = TIM6，1ms 周期中断（回调在 CODE/NEUQ_Interrupts.c 里） */
    NEUQ_PIT_init_ms(PIT_CH1, 1);
}

/* ============================ 5. 主循环（User_Main 的 while 里调用） ============================ */
void LED_Menu_Loop(void)
{
    static uint32_t last_adc_time = 0;
    uint8_t k;

    /* 按键：只改待执行方案，不影响正在运行的时序；事件由 1ms 中断产生，这里消费 */
    k = NEUQ_KEY_Get();
    if (k == KEY_UP) {
        pend_scheme = (pend_scheme == 1) ? 4 : (uint8_t)(pend_scheme - 1);
    } else if (k == KEY_DOWN) {
        pend_scheme = (pend_scheme == 4) ? 1 : (uint8_t)(pend_scheme + 1);
    }
    if (k != KEY_NONE) menu_refresh_scheme();   /* 方案有变化才刷屏（刷屏在主循环，不影响中断里的时序） */

    /* 周期切换后的同步刷新：cur_scheme 已更新、LED 已切到新方案，
     * 这里立即刷新，避免 Current 行显示滞后一个周期（看起来像 Current/Pending 反了） */
    if (scheme_changed) {
        scheme_changed = 0;
        menu_refresh_scheme();
    }

    /* 周期性刷新 PA3 电压（约 200ms 一次，HAL_GetTick 基于 SysTick 1ms 计数） */
    if ((uint32_t)(HAL_GetTick() - last_adc_time) >= 200) {
        last_adc_time = HAL_GetTick();
        menu_refresh_adc();
    }
}
