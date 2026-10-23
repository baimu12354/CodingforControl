/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_Motor
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
********************************************************************************************************************/
#include "NEUQ_Config.h"
#include "NEUQ_Motor.h"

static float s_output = 0.0f;   // 最近一次下发的输出（限幅后，未乘方向符号）

void NEUQ_Motor_Init(void)
{
#if (NEUQ_MOTOR_MODE == NEUQ_MOTOR_MODE_DUAL_PWM)
    NEUQ_PWM_init(NEUQ_MOTOR_PWM_PORT,  NEUQ_MOTOR_PWM_FREQ, 0);
    NEUQ_PWM_init(NEUQ_MOTOR_PWM2_PORT, NEUQ_MOTOR_PWM_FREQ, 0);
    NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT,  0);
    NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM2_PORT, 0);

#elif (NEUQ_MOTOR_MODE == NEUQ_MOTOR_MODE_PH_EN)
    NEUQ_PWM_init(NEUQ_MOTOR_PWM_PORT, NEUQ_MOTOR_PWM_FREQ, 0);
    NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT, 0);

    NEUQ_GPIO_init(NEUQ_MOTOR_DIR1_PORT, GPIO_Mode_Output_PushPull, GPIO_Low);

    // 手上两块驱动板的控制脚不一样：一块 PA2/PA4，另一块 PC6/PC7。
    // 两套都配成低电平，插哪块都不会出现 EN 悬空乱转。
    NEUQ_PWM_init(TIM8_CH1_PC6, NEUQ_MOTOR_PWM_FREQ, 0);
    NEUQ_PWM_set_duty(TIM8_CH1_PC6, 0);
    NEUQ_GPIO_init(PA4, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_GPIO_init(PA5, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_GPIO_init(PC7, GPIO_Mode_Output_PushPull, GPIO_Low);

#else
    NEUQ_PWM_init(NEUQ_MOTOR_PWM_PORT, NEUQ_MOTOR_PWM_FREQ, 0);
    NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT, 0);

    NEUQ_GPIO_init(NEUQ_MOTOR_DIR1_PORT, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_GPIO_init(NEUQ_MOTOR_DIR2_PORT, GPIO_Mode_Output_PushPull, GPIO_Low);
#endif

    s_output = 0.0f;
}

// 输出为 0 时只关 PWM、不动方向脚 —— 闭环在目标附近会频繁过零，
// 方向脚来回翻会有顿挫，保持上一次的电平更稳。
void NEUQ_Motor_SetOutput(float out)
{
    float v;

    if (out > NEUQ_MOTOR_PWM_MAX)  out = NEUQ_MOTOR_PWM_MAX;
    if (out < -NEUQ_MOTOR_PWM_MAX) out = -NEUQ_MOTOR_PWM_MAX;
    s_output = out;

    v = out * NEUQ_MOTOR_DIR_SIGN;

#if (NEUQ_MOTOR_MODE == NEUQ_MOTOR_MODE_DUAL_PWM)
    if (v > 0.0f)
    {
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT,  (uint32_t)v);
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM2_PORT, 0);
    }
    else if (v < 0.0f)
    {
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT,  0);
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM2_PORT, (uint32_t)(-v));
    }
    else
    {
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT,  0);
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM2_PORT, 0);
    }

#elif (NEUQ_MOTOR_MODE == NEUQ_MOTOR_MODE_PH_EN)
    {
        uint32_t duty;

        if (v > 0.0f)
        {
            NEUQ_GPIO_write(NEUQ_MOTOR_DIR1_PORT, GPIO_High);
            duty = (uint32_t)v;
        }
        else if (v < 0.0f)
        {
            NEUQ_GPIO_write(NEUQ_MOTOR_DIR1_PORT, GPIO_Low);
            duty = (uint32_t)(-v);
        }
        else
        {
            duty = 0;
        }
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT, duty);

        // 同上，两套脚的驱动板都能用
        NEUQ_GPIO_write(PA4, (v > 0.0f) ? GPIO_High : GPIO_Low);
        NEUQ_GPIO_write(PA5, (v > 0.0f) ? GPIO_High : GPIO_Low);
        NEUQ_GPIO_write(PC7, (v > 0.0f) ? GPIO_High : GPIO_Low);
        NEUQ_PWM_set_duty(TIM8_CH1_PC6, duty);
    }

#else
    {
        uint32_t duty;

        if (v > 0.0f)
        {
            NEUQ_GPIO_write(NEUQ_MOTOR_DIR1_PORT, GPIO_High);
            NEUQ_GPIO_write(NEUQ_MOTOR_DIR2_PORT, GPIO_Low);
            duty = (uint32_t)v;
        }
        else if (v < 0.0f)
        {
            NEUQ_GPIO_write(NEUQ_MOTOR_DIR1_PORT, GPIO_Low);
            NEUQ_GPIO_write(NEUQ_MOTOR_DIR2_PORT, GPIO_High);
            duty = (uint32_t)(-v);
        }
        else
        {
            NEUQ_GPIO_write(NEUQ_MOTOR_DIR1_PORT, GPIO_Low);
            NEUQ_GPIO_write(NEUQ_MOTOR_DIR2_PORT, GPIO_Low);
            duty = 0;
        }
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT, duty);
    }
#endif
}

void NEUQ_Motor_Stop(void)
{
    s_output = 0.0f;

#if (NEUQ_MOTOR_MODE == NEUQ_MOTOR_MODE_DUAL_PWM)
    NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT,  0);
    NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM2_PORT, 0);
    if (NEUQ_STOP_BRAKE)
    {
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT,  (uint32_t)NEUQ_MOTOR_PWM_MAX);
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM2_PORT, (uint32_t)NEUQ_MOTOR_PWM_MAX);
    }

#elif (NEUQ_MOTOR_MODE == NEUQ_MOTOR_MODE_PH_EN)
    // PH/EN 模式没有电气刹车：EN 拉低 = 桥臂全关，电机自己滑行
    NEUQ_GPIO_write(NEUQ_MOTOR_DIR1_PORT, GPIO_Low);
    NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT, 0);

#else
    if (NEUQ_STOP_BRAKE)
    {
        // 两输入同高即短接制动（TB6612）；L298N 还要 ENA 有效，所以 PWM 一并拉满
        NEUQ_GPIO_write(NEUQ_MOTOR_DIR1_PORT, GPIO_High);
        NEUQ_GPIO_write(NEUQ_MOTOR_DIR2_PORT, GPIO_High);
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT, (uint32_t)NEUQ_MOTOR_PWM_MAX);
    }
    else
    {
        NEUQ_GPIO_write(NEUQ_MOTOR_DIR1_PORT, GPIO_Low);
        NEUQ_GPIO_write(NEUQ_MOTOR_DIR2_PORT, GPIO_Low);
        NEUQ_PWM_set_duty(NEUQ_MOTOR_PWM_PORT, 0);
    }
#endif
}

float NEUQ_Motor_GetOutput(void)
{
    return s_output;
}
