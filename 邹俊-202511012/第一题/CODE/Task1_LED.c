/*********************************************************************************************************************
* 文件名称          Task1_LED
* 功能说明          第一题：LED 时序控制（实现），整体思路见 Task1_LED.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "Task_Config.h"
#include "Task1_LED.h"

#if (TASK_SELECT == 1)

#include "My_Key.h"
#include "My_Lcd.h"

#define T1_CYCLE_MS             (2000U)     // 每套方案的完整周期 2 s
#define T1_SCHEME_NUM           (4U)        // 方案个数

//==================================================== 方案表 ====================================================
// 每套方案由"亮、灭、亮、灭……"交替的几段组成，ms[] 依次是每段的时长，第一段一定是"亮"，
// 所有段加起来正好 2000 ms。例如方案 3：亮 200 → 灭 200 → 亮 200 → 灭 1400
typedef struct {
    uint8_t     segment_num;    // 有几段
    uint16_t    ms[4];          // 每段的时长（ms）
    const char *text;           // 屏幕上显示的说明
} T1_Scheme_Struct;

static const T1_Scheme_Struct k_scheme[T1_SCHEME_NUM] = {
    {2, {1000, 1000},            "ON1000 OFF1000"},
    {2, { 200, 1800},            "ON200 OFF1800"},
    {4, { 200,  200, 200, 1400}, "ON200 OFF200 ON200 OFF1400"},
    {4, { 200,  200, 600, 1000}, "ON200 OFF200 ON600 OFF1000"},
};

//==================================================== 变量 ====================================================
// 关键规则：s_current 和 s_time_ms 只在定时中断里修改，主循环（按键）只改 s_pending。
// 所以不管怎么按键，都改不了"当前周期走到哪里了"，当前动作不会被暂停、拉长或重置。
// volatile：告诉编译器这些变量会在中断里被改，每次使用都要重新从内存读
static volatile uint8_t  s_current = 0;     // 正在执行的方案（0 ~ 3，屏幕上显示为 1 ~ 4）
static volatile uint8_t  s_pending = 0;     // 待执行的方案，到下一个周期起点才生效
static volatile uint16_t s_time_ms = 0;     // 当前周期走到了第几毫秒（0 ~ 1999）
static volatile uint32_t s_cycle   = 0;     // 已经开始了几个周期（屏幕上显示，每 2 s 加 1）
static volatile uint8_t  s_led_on  = 0;     // LED 现在是亮（1）还是灭（0）
static volatile uint16_t s_on_ms   = 0;     // 亮灯计数：这一段亮了多少 ms；灭灯时停住不动，显示上一段亮了多久
static volatile uint16_t s_off_ms  = 0;     // 灭灯计数：这一段灭了多少 ms；亮灯时停住不动，显示上一段灭了多久
static uint8_t           s_key_timer = 0;   // 在 1 ms 中断里计数，到 10 就扫描一次按键

static uint8_t s_cursor = 0;                // 菜单光标停在哪套方案上

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查表：第 scheme 套方案在周期内第 t 毫秒时，LED 应该亮还是灭
// 返回参数     1 亮  0 灭
// 备注信息     从第一段开始把时长累加起来，看 t 落在第几段：第 0、2 段是亮，第 1、3 段是灭
//-------------------------------------------------------------------------------------------------------------------
static uint8_t t1_led_on_at(uint8_t scheme, uint16_t t)
{
    uint16_t segment_end = 0;       // 当前这一段结束的时刻
    uint8_t  i;

    for (i = 0; i < k_scheme[scheme].segment_num; i++) {
        segment_end += k_scheme[scheme].ms[i];
        if (t < segment_end) {
            if (i % 2U == 0U) {
                return 1;           // 偶数段：亮
            }
            return 0;               // 奇数段：灭
        }
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     1 ms 定时中断：控制 LED + 每 10 ms 扫描一次按键
// 备注信息     1. 周期起点（s_time_ms == 0）时，才把"待执行方案"换成"当前方案"——新方案一定从下一个周期开始
//              2. 查表得到这 1 ms LED 该亮还是灭，写到 LED 引脚
//              3. 亮灭计数：亮的这 1 ms 亮灯计数加 1，灭的这 1 ms 灭灯计数加 1；
//                 刚从灭变亮时亮灯计数先清零，刚从亮变灭时灭灯计数先清零，所以两个数就是每一段亮、灭的实际时长
//              4. 时间加 1 ms，到 2000 ms 回到 0，开始下一个周期
//-------------------------------------------------------------------------------------------------------------------
void Task1_PIT_Handler(void)
{
    uint8_t was_on = s_led_on;          // 上 1 ms LED 是亮还是灭

    if (s_time_ms == 0U) {
        s_current = s_pending;
        s_cycle++;
    }

    s_led_on = t1_led_on_at(s_current, s_time_ms);
    if (s_led_on) {
        NEUQ_GPIO_write(T1_LED_PIN, T1_LED_ON_LEVEL);
        if (!was_on) {
            s_on_ms = 0;                // 新的一段亮灯开始，从 0 重新数
        }
        s_on_ms++;                      // 灭灯计数不动
    }
    else {
        NEUQ_GPIO_write(T1_LED_PIN, T1_LED_OFF_LEVEL);
        if (was_on) {
            s_off_ms = 0;               // 新的一段灭灯开始，从 0 重新数
        }
        s_off_ms++;                     // 亮灯计数不动
    }

    s_time_ms++;
    if (s_time_ms >= T1_CYCLE_MS) {
        s_time_ms = 0;
    }

    s_key_timer++;
    if (s_key_timer >= KEY_SCAN_PERIOD_MS) {
        s_key_timer = 0;
        My_Key_scan();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理按键（主循环中调用）
// 备注信息     上    ：光标上移（到顶回到最下面），按住可以连续移动
//              下    ：光标下移（到底回到最上面），按住可以连续移动
//              确认  ：把光标所在方案设为"待执行"，当前周期跑完后自动切换
//              返回  ：撤销待执行（待执行 = 当前方案）
// 返回参数     1 表示有按键，需要马上刷新屏幕
//-------------------------------------------------------------------------------------------------------------------
static uint8_t t1_handle_keys(void)
{
    Key_Event_Struct event;
    uint8_t          changed = 0;

    while (My_Key_get_event(&event)) {
        changed = 1;

        if (event.id == KEY_UP) {
            if (s_cursor == 0U) {
                s_cursor = T1_SCHEME_NUM - 1U;
            }
            else {
                s_cursor--;
            }
        }
        else if (event.id == KEY_DOWN) {
            s_cursor++;
            if (s_cursor >= T1_SCHEME_NUM) {
                s_cursor = 0;
            }
        }
        else if (event.id == KEY_OK) {
            s_pending = s_cursor;
        }
        else if (event.id == KEY_BACK) {
            // 要先读 s_current 再写 s_pending，如果两步之间正好到了周期起点，s_current 会被中断改掉，
            // 所以这两步之间关一下中断（只有几条指令，远小于 1 us，不影响 1 ms 节拍）
            __disable_irq();
            s_pending = s_current;
            __enable_irq();
            s_cursor = s_pending;
        }
    }
    return changed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     刷新屏幕（主循环中调用；My_Lcd 只重画变了的字符，所以很快）
// 备注信息     行 0      标题
//              行 1      表头
//              行 2 ~ 5  四套方案：蓝底 = 光标，绿字 + RUN = 正在执行，NXT = 待执行
//              行 6      当前方案 / 待执行方案（两者不同时显示红字 next）
//              行 7      周期计数、周期内时间、LED 亮灭
//              行 8      亮灯计数、灭灯计数（ms），正在数的那个在变，另一个停在上一段的时长
//              行 9      操作提示
//-------------------------------------------------------------------------------------------------------------------
static void t1_draw_page(void)
{
    uint8_t current = s_current;        // 中断随时可能改这两个变量，先读出来，保证这一屏前后一致
    uint8_t pending = s_pending;
    uint8_t i;

    My_Lcd_print(0, UI_STYLE_TITLE, " Task1  LED Timing Control");
    My_Lcd_print(1, UI_STYLE_HINT,  "No Pattern(ms)       cycle=2s");

    for (i = 0; i < T1_SCHEME_NUM; i++) {
        const char   *tag   = "";
        UI_Style_Enum style = UI_STYLE_NORMAL;

        if (i == current) {
            tag   = "RUN";
            style = UI_STYLE_GOOD;
        }
        else if (i == pending) {
            tag = "NXT";
        }
        if (i == s_cursor) {
            style = UI_STYLE_SELECT;    // 光标所在行优先显示为蓝底
        }
        My_Lcd_print((uint8_t)(2U + i), style, "%u %-26s %-3s", (unsigned int)(i + 1U), k_scheme[i].text, tag);
    }

    if (pending != current) {
        My_Lcd_print(6, UI_STYLE_WARN, "Current:%u   Pending:%u (next)", (unsigned int)(current + 1U), (unsigned int)(pending + 1U));
    }
    else {
        My_Lcd_print(6, UI_STYLE_NORMAL, "Current:%u   Pending:%u", (unsigned int)(current + 1U), (unsigned int)(pending + 1U));
    }
    My_Lcd_print(7, UI_STYLE_NORMAL, "Cycle#%-6lu t=%4ums LED:%s",
                  (unsigned long)s_cycle, (unsigned int)s_time_ms, s_led_on ? "ON" : "OFF");
    My_Lcd_print(8, UI_STYLE_NORMAL, "ON:%4u ms       OFF:%4u ms", (unsigned int)s_on_ms, (unsigned int)s_off_ms);
    My_Lcd_print(9, UI_STYLE_HINT,   "UP/DN:move OK:set BACK:undo");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     第一题主函数
// 备注信息     先启动 1 ms 定时中断，上电后 LED 立刻按方案 1 循环；LCD 之后再初始化。
//              主循环用 HAL_GetTick()（系统毫秒计数）判断时间：每 100 ms 刷新一次屏幕
//-------------------------------------------------------------------------------------------------------------------
void Task1_Main(void)
{
    uint32_t last_ui_tick;

    NEUQ_Debug_init();                                  // 调试串口 UART_0（LCD 初始化时库里会用 printf，必须先初始化）
    NEUQ_Delay_init();

    NEUQ_GPIO_init(T1_LED_PIN, GPIO_Mode_Output_PushPull, T1_LED_OFF_LEVEL);
    My_Key_init();
    NEUQ_PIT_init_ms(PIT_CH1, 1);                       // 1 ms 定时中断，从这里开始 LED 独立运行

    My_Lcd_init();
    t1_draw_page();

    last_ui_tick = HAL_GetTick();
    while (1) {
        uint32_t now    = HAL_GetTick();
        uint8_t  redraw = t1_handle_keys();

        if (redraw || now - last_ui_tick >= 100U) {     // 有按键立即刷新，否则每 100 ms 刷新
            last_ui_tick = now;
            t1_draw_page();
        }
    }
}

#endif
