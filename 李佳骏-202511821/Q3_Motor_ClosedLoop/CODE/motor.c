/* motor.c - 编码器测速与增量 PID */

#include "motor.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "board_config.h"

/* 中断与主循环共享的状态。整型和对齐的 32 位浮点在 Cortex-M4 上读写是原子的，
 * 所以这里不需要临界区。 */
static volatile uint8_t s_enabled     = 0;      /* 上电默认停止，等上位机启动 */
static volatile float   s_target_rpm  = 0.0f;
static volatile float   s_actual_rpm  = 0.0f;
static volatile float   s_kp          = T3_PID_KP_DEFAULT;
static volatile float   s_ki          = T3_PID_KI_DEFAULT;
static volatile float   s_kd          = T3_PID_KD_DEFAULT;
static volatile int32_t s_pwm_out     = 0;
static volatile int32_t s_counts_last = 0;
static volatile uint8_t s_reset_req   = 0;      /* 主循环请求中断复位 PID 历史 */

/* PID 历史只由中断读写 */
static IncrementalPID   s_pid;

/* 上一次的方向电平。输出为零时保持方向不变，只把占空比拉零。 */
static GPIO_Level       s_dir_level = GPIO_High;

/* 前置声明：Motor_ResetPidState 会先于定义调用它 */
static void Motor_ApplyOutput(int32_t output);

#if (T3_MOTOR_PWM_USE_TIM8 != 0)
static TIM_HandleTypeDef s_tim8_pwm;

static void Motor_PwmInit_TIM8(void)
{
    GPIO_InitTypeDef   gpio = {0};
    TIM_OC_InitTypeDef oc   = {0};

    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_6;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &gpio);

    s_tim8_pwm.Instance               = TIM8;
    s_tim8_pwm.Init.Prescaler         = 1;
    s_tim8_pwm.Init.Period            = PWM_MAX - 1;
    s_tim8_pwm.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_tim8_pwm.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_tim8_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&s_tim8_pwm);

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&s_tim8_pwm, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&s_tim8_pwm, TIM_CHANNEL_1);
}

static void Motor_PwmSetDuty(uint32_t duty)
{
    __HAL_TIM_SET_COMPARE(&s_tim8_pwm, TIM_CHANNEL_1, duty);
}
#else
static void Motor_PwmSetDuty(uint32_t duty)
{
    NEUQ_PWM_set_duty(T3_MOTOR_PWM_PORT, duty);
}
#endif

static void Motor_ResetPidState(void)
{
    s_pid.error[0] = 0.0f;
    s_pid.error[1] = 0.0f;
    s_pid.error[2] = 0.0f;
    s_pwm_out      = 0;
    Motor_ApplyOutput(0);
}

/* 有符号输出 → 方向 + 占空比 */
static void Motor_ApplyOutput(int32_t output)
{
    int32_t  scaled;
    uint32_t duty;
    GPIO_Level dir_level;

    scaled = (int32_t)((float)output * T3_MOTOR_DIRECTION);

    if (scaled > (int32_t)PWM_MAX)
    {
        scaled = (int32_t)PWM_MAX;
    }
    else if (scaled < -(int32_t)PWM_MAX)
    {
        scaled = -(int32_t)PWM_MAX;
    }

    /* 先定方向，再把大小当占空比送进 EN。
     * 正数走一个方向、负数走另一个，零就只把占空比拉零、方向保持不变。 */
    if (scaled > 0)
    {
        duty      = (uint32_t)scaled;
        dir_level = GPIO_High;
    }
    else if (scaled < 0)
    {
        duty      = (uint32_t)(-scaled);
        dir_level = GPIO_Low;
    }
    else
    {
        duty      = 0U;
        dir_level = s_dir_level;        /* 保持上一次的方向，只把速度降到零 */
    }

    s_dir_level = dir_level;
    NEUQ_GPIO_write(T3_MOTOR_DIR_PIN, dir_level);
    Motor_PwmSetDuty(duty);
}

void Motor_Init(void)
{
    /* 不用 Flash，片选拉高避免 SPI 引脚浮空 */
    NEUQ_GPIO_init(T3_FLASH_CS_PIN, GPIO_Mode_Output_PushPull, GPIO_High);

    /* 方向脚先给一个确定电平，再初始化编码器和 PWM */
    NEUQ_GPIO_init(T3_MOTOR_DIR_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
    s_dir_level = GPIO_Low;

    NEUQ_Encoder_init(T3_ENCODER_DEV, T3_ENCODER_PHASE_A, T3_ENCODER_PHASE_B);
    NEUQ_Encoder_Clear(T3_ENCODER_DEV);

#if (T3_MOTOR_PWM_USE_TIM8 != 0)
    Motor_PwmInit_TIM8();
#else
    NEUQ_PWM_init(T3_MOTOR_PWM_PORT, T3_PWM_FREQ_HZ, 0);
#endif

    s_pid.Kp            = T3_PID_KP_DEFAULT;
    s_pid.Ki            = T3_PID_KI_DEFAULT;
    s_pid.Kd            = T3_PID_KD_DEFAULT;
    s_pid.max_delta_pwm = T3_PID_MAX_DELTA;
    s_pid.error[0]      = 0.0f;
    s_pid.error[1]      = 0.0f;
    s_pid.error[2]      = 0.0f;

    s_enabled     = 0;
    s_target_rpm  = 0.0f;
    s_actual_rpm  = 0.0f;
    s_kp          = T3_PID_KP_DEFAULT;
    s_ki          = T3_PID_KI_DEFAULT;
    s_kd          = T3_PID_KD_DEFAULT;
    s_pwm_out     = 0;
    s_counts_last = 0;
    s_reset_req   = 0;

    Motor_ApplyOutput(0);
}

/* TIM7 每 10 ms 调用：测速 + 增量 PID + PWM */
void Motor_ControlIsr(void)
{
    float   counts;
    float   rpm;
    float   delta;
    float   output;

    /* 1. 测速 */
    /* 先读后清：读到的是本周期内的增量，清零后下一周期从零开始计 */
    counts = (float)NEUQ_Encoder_Read(T3_ENCODER_DEV);
    NEUQ_Encoder_Clear(T3_ENCODER_DEV);
    s_counts_last = (int32_t)counts;

    /* 转速(rpm) = 计数 / 每转计数 × 每秒周期数 × 60 */
    rpm = counts * T3_ENCODER_DIRECTION
          * (60000.0f / ((float)T3_CONTROL_PERIOD_MS * T3_ENCODER_COUNT_PER_REV));
    s_actual_rpm = rpm;

    /* 2. 处理主循环的复位请求 */
    if (s_reset_req != 0U)
    {
        s_reset_req = 0;
        Motor_ResetPidState();
    }

    /* 3. 闭环 */
    if (s_enabled == 0U)
    {
        /* 停止状态：输出保持为零，不参与调节 */
        if (s_pwm_out != 0)
        {
            s_pwm_out = 0;
            Motor_ApplyOutput(0);
        }
        return;
    }

    s_pid.Kp = s_kp;
    s_pid.Ki = s_ki;
    s_pid.Kd = s_kd;

    delta  = IncrementalPID_Update(&s_pid, s_target_rpm, rpm);
    output = (float)s_pwm_out + delta;

    if (output > (float)PWM_MAX)
    {
        output = (float)PWM_MAX;
    }
    else if (output < -(float)PWM_MAX)
    {
        output = -(float)PWM_MAX;
    }

    s_pwm_out = (int32_t)output;
    Motor_ApplyOutput(s_pwm_out);
}

void Motor_Start(void)
{
    s_reset_req  = 1;       /* 起步前清掉历史，避免带着上次的积分冲一下 */
    s_enabled    = 1;
}

void Motor_Stop(void)
{
    s_enabled    = 0;
    s_target_rpm = 0.0f;
    s_reset_req  = 1;
}

void Motor_SetTargetRpm(float rpm)
{
    if (rpm > T3_TARGET_RPM_MAX)
    {
        rpm = T3_TARGET_RPM_MAX;
    }
    else if (rpm < -T3_TARGET_RPM_MAX)
    {
        rpm = -T3_TARGET_RPM_MAX;
    }
    s_target_rpm = rpm;
}

void Motor_SetPid(float kp, float ki, float kd)
{
    s_kp = kp;
    s_ki = ki;
    s_kd = kd;
    s_reset_req = 1;
}

void Motor_GetSnapshot(motor_snapshot_t *out)
{
    if (out == 0)
    {
        return;
    }

    out->enabled     = s_enabled;
    out->target_rpm  = s_target_rpm;
    out->actual_rpm  = s_actual_rpm;
    out->kp          = s_kp;
    out->ki          = s_ki;
    out->kd          = s_kd;
    out->pwm_out     = s_pwm_out;
    out->counts_last = s_counts_last;
}

uint8_t Motor_IsEnabled(void)
{
    return s_enabled;
}
