#include "NEUQ_lib_include.h"

volatile uint8_t TASK_CMD = 1;

void LED_RUN_init(void)
{
    NEUQ_PWM_init(TIM1_CH1_PA8, 1000, 10000);
}
void LED_TASK(LED_MODE_ID id)
{
    static uint16_t elapsed_ms = 0;
    elapsed_ms++;
    switch (id)
    {
    case LED_MODE_SLOW_BLINK:
    {
        if (elapsed_ms == 1000)
        {
            NEUQ_PWM_set_duty(TIM1_CH1_PA8, 0);
        }
        if (elapsed_ms >= 2000)
        {
            elapsed_ms = 0;
            current = TASK_CMD;
            NEUQ_PWM_set_duty(TIM1_CH1_PA8, 10000);
        }
        break;
    }
        case LED_MODE_SINGLE_FLASH:
    {
        if (elapsed_ms == 200)
        {
            NEUQ_PWM_set_duty(TIM1_CH1_PA8, 0);
        }
        if (elapsed_ms >= 2000)
        {
            elapsed_ms = 0;
            current = TASK_CMD;
            NEUQ_PWM_set_duty(TIM1_CH1_PA8, 10000);
        }
        break;
    }
        case LED_MODE_DOUBLE_FLASH:
    {
       if (elapsed_ms < 200 ||
             (elapsed_ms >= 400 && elapsed_ms < 600))
        {
             NEUQ_PWM_set_duty(TIM1_CH1_PA8, 10000);
        }
        else
        {
         NEUQ_PWM_set_duty(TIM1_CH1_PA8, 0);
        }
        if (elapsed_ms >= 2000)
        {
            elapsed_ms = 0;
            current = TASK_CMD;
            NEUQ_PWM_set_duty(TIM1_CH1_PA8, 10000);
        }
        break;
    }
        case LED_MODE_SHORT_LONG_FLASH:
    {
        if (elapsed_ms < 200 ||
             (elapsed_ms >= 400 && elapsed_ms < 1000))
        {
             NEUQ_PWM_set_duty(TIM1_CH1_PA8, 10000);
        }
        else
        {
         NEUQ_PWM_set_duty(TIM1_CH1_PA8, 0);
        }
        if (elapsed_ms >= 2000)
        {
            elapsed_ms = 0;
            current = TASK_CMD;
            NEUQ_PWM_set_duty(TIM1_CH1_PA8, 10000);
        }
        break;
    }
    default:
        break;
    }
}
