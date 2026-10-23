/*********************************************************************************************************************
* 文件名称          My_Key
* 功能说明          菜单按键（实现），说明见 My_Key.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "Task_Config.h"
#include "My_Key.h"

#define KEY_QUEUE_SIZE      (16U)       // 事件队列长度（最多存 15 个，留一个空位用来区分"满"和"空"）

// 四个键的引脚和"按下时的电平"，顺序与 Key_Id_Enum 一致；具体引脚在 Task_Config.h 中修改
static const GPIO_Ports_ENUM s_pin[KEY_NUM]    = {KEY_UP_PIN, KEY_DOWN_PIN, KEY_OK_PIN, KEY_BACK_PIN};
static const GPIO_Level      s_active[KEY_NUM] = {KEY_UP_ACTIVE_LEVEL, KEY_DOWN_ACTIVE_LEVEL,
                                                  KEY_OK_ACTIVE_LEVEL, KEY_BACK_ACTIVE_LEVEL};

// 每个键的消抖状态
static uint8_t  s_stable[KEY_NUM];      // 消抖后认定的状态：1 按下，0 松开
static uint8_t  s_count[KEY_NUM];       // "读到的状态和认定的状态不同"已经连续了几次
static uint16_t s_hold_ms[KEY_NUM];     // 已经按住多久（用于长按连发）

// 事件队列（环形缓冲）：只有中断写 s_head，只有主循环写 s_tail，两边各改各的，不需要关中断
static volatile Key_Event_Struct s_queue[KEY_QUEUE_SIZE];
static volatile uint8_t          s_head = 0;    // 下一个事件写到哪里
static volatile uint8_t          s_tail = 0;    // 下一个事件从哪里读

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     往队列里放一个事件；队列满了就丢掉这个新事件（主循环卡住时不会覆盖旧事件）
//-------------------------------------------------------------------------------------------------------------------
static void key_push(uint8_t id, uint8_t type)
{
    uint8_t next = (uint8_t)((s_head + 1U) % KEY_QUEUE_SIZE);
    if (next == s_tail) {
        return;
    }
    s_queue[s_head].id   = id;
    s_queue[s_head].type = type;
    s_head = next;                      // 先写数据再移动 head，主循环看到 head 变化时数据一定已经写好
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化按键引脚
// 备注信息     按下是低电平的键配置成内部上拉（松开时被拉高），按下是高电平的键配置成内部下拉，
//              这样按键悬空（没接）时读到的都是"松开"
//-------------------------------------------------------------------------------------------------------------------
void My_Key_init(void)
{
    uint8_t i;
    for (i = 0; i < KEY_NUM; i++) {
        GPIO_Mode_ENUM mode = (GPIO_Mode_ENUM)((s_active[i] == GPIO_Low) ? GPIO_Mode_Input_PullUp : GPIO_Mode_Input_PullDown);
        NEUQ_GPIO_init(s_pin[i], mode, GPIO_High);
        s_stable[i]  = 0;
        s_count[i]   = 0;
        s_hold_ms[i] = 0;
    }
    s_head = 0;
    s_tail = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     扫描一次所有按键（在定时中断里每 KEY_SCAN_PERIOD_MS 调用一次）
// 备注信息     消抖原理：机械按键按下/松开的瞬间触点会抖动几毫秒，读到的电平忽高忽低。
//              只有"读到的状态"连续 KEY_DEBOUNCE_COUNT 次（默认 3 次 = 30 ms）都与"认定的状态"不同，
//              才承认按键真的变了；中间只要有一次读回原状态，计数就清零重来，抖动因此被滤掉。
//              连发原理：上/下键认定按下后开始计时，按住满 KEY_REPEAT_DELAY_MS 产生第一次连发，
//              之后每隔 KEY_REPEAT_PERIOD_MS 再产生一次。
//-------------------------------------------------------------------------------------------------------------------
void My_Key_scan(void)
{
    uint8_t i;
    for (i = 0; i < KEY_NUM; i++) {
        uint8_t pressed = (NEUQ_GPIO_read(s_pin[i]) == s_active[i]) ? 1U : 0U;   // 这一次读到的状态

        if (pressed != s_stable[i]) {
            // 读到的和认定的不一样：可能真的按下/松开了，也可能只是抖动，先计数
            s_count[i]++;
            if (s_count[i] >= KEY_DEBOUNCE_COUNT) {
                s_stable[i]  = pressed;         // 连续多次都这样，认定状态真的变了
                s_count[i]   = 0;
                s_hold_ms[i] = 0;
                if (pressed) {
                    key_push(i, KEY_EVENT_PRESS);
                }
            }
        }
        else {
            // 读到的和认定的一样：之前的变化只是抖动，计数清零
            s_count[i] = 0;

            // 上/下键一直按着：计时，到时间就产生连发事件
            if (s_stable[i] && (i == KEY_UP || i == KEY_DOWN)) {
                s_hold_ms[i] += KEY_SCAN_PERIOD_MS;
                if (s_hold_ms[i] >= KEY_REPEAT_DELAY_MS) {
                    key_push(i, KEY_EVENT_REPEAT);
                    s_hold_ms[i] = KEY_REPEAT_DELAY_MS - KEY_REPEAT_PERIOD_MS;  // 再过一个连发间隔触发下一次
                }
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主循环取出一个按键事件
// 返回参数     1 取到事件  0 队列为空
//-------------------------------------------------------------------------------------------------------------------
uint8_t My_Key_get_event(Key_Event_Struct *event)
{
    if (s_tail == s_head) {
        return 0;
    }
    event->id   = s_queue[s_tail].id;
    event->type = s_queue[s_tail].type;
    s_tail = (uint8_t)((s_tail + 1U) % KEY_QUEUE_SIZE);
    return 1;
}

void My_Key_clear_events(void)
{
    s_tail = s_head;
}
