/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分，遵循 GPL-3.0 或更高版本许可。
*********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

#define LED_PIN             PA8
#define KEY_PIN             PA0
#define KEY_DEBOUNCE_MS     20UL

typedef struct {
    uint16_t duration_ms;
    uint8_t led_on;
} LedStep;

static const LedStep plan1[] = {{1000, 1}, {1000, 0}};
static const LedStep plan2[] = {{200, 1}, {1800, 0}};
static const LedStep plan3[] = {{200, 1}, {200, 0}, {200, 1}, {1400, 0}};
static const LedStep plan4[] = {{200, 1}, {200, 0}, {600, 1}, {1000, 0}};
static const LedStep *const plans[] = {plan1, plan2, plan3, plan4};
static const uint8_t plan_step_count[] = {2, 2, 4, 4};

void User_Main(void)
{
    uint8_t current_plan = 0;
    uint8_t pending_plan = 0;
    uint8_t step = 0;
    uint8_t key_candidate = 1;
    uint8_t key_stable = 1;
    uint8_t shown_current = 0xFF;
    uint8_t shown_pending = 0xFF;
    uint32_t key_changed_at;
    uint32_t step_started;
    char line[24];

    NEUQ_Debug_init();
    NEUQ_Delay_init();
    NEUQ_GPIO_init(LED_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_GPIO_init(KEY_PIN, GPIO_Mode_Input_PullUp, GPIO_High);

    NEUQ_LCD200_init();
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(8, 12, "LED Pattern Menu");
    NEUQ_LCD200_DisplayString(8, 64, "KEY PA0: NEXT PLAN");
    NEUQ_LCD200_DisplayString(8, 100, "RUN:");
    NEUQ_LCD200_DisplayString(8, 140, "NEXT:");

    step_started = HAL_GetTick();
    key_changed_at = step_started;

    while (1) {
        uint32_t now = HAL_GetTick();
        uint8_t key_raw = (NEUQ_GPIO_read(KEY_PIN) == GPIO_High) ? 1U : 0U;

        /* 输入连续稳定20ms后确认；长按只在按下沿触发一次。 */
        if (key_raw != key_candidate) {
            key_candidate = key_raw;
            key_changed_at = now;
        } else if ((key_candidate != key_stable) &&
                   ((uint32_t)(now - key_changed_at) >= KEY_DEBOUNCE_MS)) {
            key_stable = key_candidate;
            if (key_stable == 0U) {
                pending_plan = (uint8_t)((pending_plan + 1U) % 4U);
            }
        }

        /* 按绝对时间推进阶段，刷新菜单不会重置闪烁计时。 */
        while ((uint32_t)(now - step_started) >= plans[current_plan][step].duration_ms) {
            step_started += plans[current_plan][step].duration_ms;
            ++step;
            if (step >= plan_step_count[current_plan]) {
                step = 0;
                current_plan = pending_plan;
            }
        }
        NEUQ_GPIO_write(LED_PIN,
                        plans[current_plan][step].led_on ? GPIO_High : GPIO_Low);

        if (shown_current != current_plan) {
            (void)sprintf(line, "RUN: PLAN %u   ",
                          (unsigned int)(current_plan + 1U));
            NEUQ_LCD200_DisplayString(8, 100, line);
            shown_current = current_plan;
        }
        if (shown_pending != pending_plan) {
            (void)sprintf(line, "NEXT: PLAN %u   ",
                          (unsigned int)(pending_plan + 1U));
            NEUQ_LCD200_DisplayString(8, 140, line);
            shown_pending = pending_plan;
        }
    }
}
