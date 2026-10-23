#include "NEUQ_lib_include.h"

volatile uint32_t led_tick = 0;

static uint8_t current_plan = 1;
static uint8_t pending_plan = 1;
static uint32_t plan_start_tick = 0;
static uint32_t step_start_tick = 0;
static uint8_t step_index = 0;

typedef struct
{
    uint16_t on_time;
    uint16_t off_time;
} LedStep;

static const LedStep plans[NEUQ_LED_PLAN_COUNT][2] =
{
    {{1000, 1000}, {0, 0}},
    {{ 200, 1800}, {0, 0}},
    {{ 200,  200}, {200, 1400}},
    {{ 200,  200}, {600, 1000}}
};

static const uint8_t plan_step_count[NEUQ_LED_PLAN_COUNT] = {1, 1, 2, 2};

void NEUQ_Led_Init(void)
{
    NEUQ_GPIO_init(PA8, GPIO_Mode_Output_PushPull, GPIO_High);
    plan_start_tick = led_tick;
    step_start_tick = led_tick;
    step_index = 0;
}

void NEUQ_Led_Tick_Handler(void)
{
    led_tick++;
}

void NEUQ_Led_Task(void)
{
    uint32_t now = led_tick;
    uint8_t plan_index = current_plan - 1;
    uint8_t phase_count = plan_step_count[plan_index] * 2;
    uint8_t step;
    uint16_t duration;

    if (now - plan_start_tick >= 2000)
    {
        plan_start_tick = now;
        step_start_tick = now;
        step_index = 0;
        current_plan = pending_plan;
        NEUQ_GPIO_write(PA8, GPIO_High);
        return;
    }

    if (step_index >= phase_count)
    {
        return;
    }

    step = step_index / 2;
    duration = (step_index % 2 == 0)
             ? plans[plan_index][step].on_time
             : plans[plan_index][step].off_time;

    if (now - step_start_tick < duration)
    {
        return;
    }

    step_start_tick = now;
    step_index++;

    if (step_index < phase_count)
    {
        NEUQ_GPIO_write(PA8, (step_index % 2 == 0) ? GPIO_High : GPIO_Low);
    }
}

void NEUQ_Led_SetPendingPlan(uint8_t plan)
{
    if (plan >= 1 && plan <= NEUQ_LED_PLAN_COUNT)
    {
        pending_plan = plan;
    }
}

uint8_t NEUQ_Led_GetCurrentPlan(void)
{
    return current_plan;
}

uint8_t NEUQ_Led_GetPendingPlan(void)
{
    return pending_plan;
}
