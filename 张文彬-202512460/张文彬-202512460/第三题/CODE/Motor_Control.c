#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

/* ==================== 对主循环公开的控制状态 ==================== */
static volatile uint8_t s_motor_running = 0U;
static volatile float s_target_rpm = 0.0f;
static volatile float s_actual_rpm = 0.0f;
static volatile float s_pid_kp = MOTOR_DEFAULT_KP;
static volatile float s_pid_ki = MOTOR_DEFAULT_KI;
static volatile float s_pid_kd = MOTOR_DEFAULT_KD;
static volatile uint8_t s_motor_fault = MOTOR_CONTROL_FAULT_NONE;
static volatile int32_t s_encoder_total_count = 0;

/* ==================== 只在 10 ms 控制中断内使用的状态 ==================== */
static float s_pid_integral = 0.0f;
static float s_pid_last_speed = 0.0f;
static float s_filtered_rpm = 0.0f;
static uint8_t s_filter_ready = 0U;
static uint8_t s_pid_measurement_ready = 0U;
static uint32_t s_encoder_watch_cycles = 0UL;
static uint32_t s_encoder_watch_counts = 0UL;
static uint32_t s_wrong_direction_cycles = 0UL;

/* ==================== 内部小工具 ==================== */
static float Motor_ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float Motor_AbsFloat(float value)
{
    return value >= 0.0f ? value : -value;
}

#if (MOTOR_ENABLE_NO_ENCODER_AUTO_STOP != 0U)
static uint32_t Motor_AbsCount(int32_t value)
{
    if (value >= 0) return (uint32_t)value;
    return (uint32_t)(-(int64_t)value);
}
#endif

static uint32_t Motor_EnterCritical(void)
{
    uint32_t previous_mask = __get_PRIMASK();
    __disable_irq();
    return previous_mask;
}

static void Motor_ExitCritical(uint32_t previous_mask)
{
    if (previous_mask == 0U) __enable_irq();
}

static void Motor_ResetPidState(void)
{
    s_pid_integral = 0.0f;
    s_pid_last_speed = 0.0f;
    s_pid_measurement_ready = 0U;
    s_encoder_watch_cycles = 0UL;
    s_encoder_watch_counts = 0UL;
    s_wrong_direction_cycles = 0UL;
}

static void Motor_ApplyOutput(float direction, float duty)
{
    uint32_t duty_count;

    duty = Motor_ClampFloat(duty, 0.0f, (float)PWM_MAX);
    duty_count = (uint32_t)(duty + 0.5f);

    if (duty_count == 0U)
    {
        NEUQ_PWM_set_duty(MOTOR_PWM_PORT, 0U);
        return;
    }

    NEUQ_GPIO_write(MOTOR_DIRECTION_PIN,
                    direction >= 0.0f ? GPIO_High : GPIO_Low);
    NEUQ_PWM_set_duty(MOTOR_PWM_PORT, duty_count);
}

static float Motor_GetFeedforward(float target_speed)
{
#if (MOTOR_ENABLE_SPEED_FEEDFORWARD != 0U)
    return Motor_ClampFloat(target_speed * (float)PWM_MAX /
                            MOTOR_NOMINAL_NO_LOAD_RPM,
                            0.0f,
                            (float)PWM_MAX);
#else
    (void)target_speed;
    return 0.0f;
#endif
}

#if ((MOTOR_ENABLE_NO_ENCODER_AUTO_STOP != 0U) || \
     (MOTOR_ENABLE_WRONG_DIR_AUTO_STOP != 0U))
static void Motor_LatchFault(uint8_t fault)
{
    s_motor_fault = fault;
    s_motor_running = 0U;
    s_target_rpm = 0.0f;
    Motor_ResetPidState();
    Motor_ApplyOutput(1.0f, 0.0f);
}
#endif

/* ==================== 初始化与命令接口 ==================== */
void Motor_Control_Init(void)
{
    NEUQ_GPIO_init(MOTOR_DIRECTION_PIN,
                   GPIO_Mode_Output_PushPull,
                   GPIO_Low);
    NEUQ_PWM_init(MOTOR_PWM_PORT,
                  MOTOR_PWM_HZ,
                  MOTOR_PWM_INITIAL_DUTY);
    NEUQ_Encoder_init(MOTOR_ENCODER,
                      MOTOR_ENCODER_PHASE_A,
                      MOTOR_ENCODER_PHASE_B);

    s_target_rpm = 0.0f;
    s_actual_rpm = 0.0f;
    s_filtered_rpm = 0.0f;
    s_filter_ready = 0U;
    s_motor_fault = MOTOR_CONTROL_FAULT_NONE;
    s_encoder_total_count = 0;
    Motor_Control_SetRunning(0U);
}

void Motor_Control_SetRunning(uint8_t running)
{
    uint32_t previous_mask = Motor_EnterCritical();

    Motor_ResetPidState();
    if (running != 0U && s_motor_fault == MOTOR_CONTROL_FAULT_NONE)
    {
        s_motor_running = 1U;
    }
    else
    {
        s_motor_running = 0U;
        Motor_ApplyOutput(1.0f, 0.0f);
    }

    Motor_ExitCritical(previous_mask);
}

void Motor_Control_SetTargetRpm(float target_rpm)
{
    uint32_t previous_mask;
    float old_target;

    target_rpm = Motor_ClampFloat(target_rpm,
                                  -MOTOR_MAX_TARGET_RPM,
                                  MOTOR_MAX_TARGET_RPM);
    if (Motor_AbsFloat(target_rpm) <= MOTOR_TARGET_ZERO_DEADBAND_RPM)
    {
        target_rpm = 0.0f;
    }

    previous_mask = Motor_EnterCritical();
    old_target = s_target_rpm;
    s_target_rpm = target_rpm;

    /* 换向或目标清零时清除旧积分，避免旧积分继续推动原方向。 */
    if (target_rpm == 0.0f || old_target * target_rpm < 0.0f)
    {
        Motor_ResetPidState();
        Motor_ApplyOutput(1.0f, 0.0f);
    }
    Motor_ExitCritical(previous_mask);
}

void Motor_Control_SetPid(float kp, float ki, float kd)
{
    uint32_t previous_mask = Motor_EnterCritical();

    s_pid_kp = kp;
    s_pid_ki = ki;
    s_pid_kd = kd;
    Motor_ResetPidState();

    Motor_ExitCritical(previous_mask);
}

uint8_t Motor_Control_IsRunning(void)
{
    return s_motor_running;
}

uint8_t Motor_Control_HasFault(void)
{
    return (uint8_t)(s_motor_fault != MOTOR_CONTROL_FAULT_NONE);
}

void Motor_Control_ClearFault(void)
{
    uint32_t previous_mask = Motor_EnterCritical();

    if (s_motor_running == 0U)
    {
        s_motor_fault = MOTOR_CONTROL_FAULT_NONE;
        Motor_ResetPidState();
        Motor_ApplyOutput(1.0f, 0.0f);
    }

    Motor_ExitCritical(previous_mask);
}

void Motor_Control_ResetEncoderTotal(void)
{
    uint32_t previous_mask = Motor_EnterCritical();

    s_encoder_total_count = 0;

    Motor_ExitCritical(previous_mask);
}

void Motor_Control_GetStatus(Motor_Control_Status *status)
{
    uint32_t previous_mask;

    if (status == NULL) return;

    /* 一次性复制，避免 10 ms 中断恰好在复制一半时更新数据。 */
    previous_mask = Motor_EnterCritical();
    status->target_rpm = s_target_rpm;
    status->actual_rpm = s_actual_rpm;
    status->kp = s_pid_kp;
    status->ki = s_pid_ki;
    status->kd = s_pid_kd;
    status->running = s_motor_running;
    status->fault = s_motor_fault;
    status->encoder_total_count = s_encoder_total_count;
    Motor_ExitCritical(previous_mask);
}

/* ==================== 10 ms：测速、PID、PWM ==================== */
void Motor_Control_Update10ms(void)
{
    int32_t encoder_count;
    float raw_rpm;
    float target_speed;
    float actual_speed;
    float direction;
    float error;
    float derivative;
    float candidate_integral;
    float feedforward;
    float output_unlimited;
    float output;

    encoder_count = NEUQ_Encoder_Read(MOTOR_ENCODER);
    NEUQ_Encoder_Clear(MOTOR_ENCODER);

    /*
     * 硬件计数器每 10 ms 清零，但本段计数先累加到软件变量。
     * STOP 清零后，按电机正向手动转输出轴一圈，I10 应接近 +440。
     */
    s_encoder_total_count += MOTOR_ENCODER_COUNT_DIRECTION * encoder_count;

    raw_rpm = MOTOR_ENCODER_DIRECTION * (float)encoder_count * 60.0f /
              (MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV * MOTOR_CONTROL_DT);

    if (s_filter_ready == 0U)
    {
        s_filtered_rpm = raw_rpm;
        s_filter_ready = 1U;
    }
    else
    {
        s_filtered_rpm += MOTOR_SPEED_FILTER_ALPHA *
                          (raw_rpm - s_filtered_rpm);
    }
    s_actual_rpm = s_filtered_rpm;

    target_speed = Motor_AbsFloat(s_target_rpm);
    direction = s_target_rpm >= 0.0f ? 1.0f : -1.0f;

    /*
     * 本题控制的是转速大小，电机方向由目标值正负和 DIR 引脚决定。
     * PID 使用速度绝对值，不让 AB 相顺序或编码器方向符号把正常转动
     * 误当成“负速度”，从而导致输出长期饱和或触发错误停机。
     */
    actual_speed = Motor_AbsFloat(s_actual_rpm);

    if (s_motor_running == 0U ||
        s_motor_fault != MOTOR_CONTROL_FAULT_NONE ||
        target_speed <= MOTOR_TARGET_ZERO_DEADBAND_RPM)
    {
        Motor_ResetPidState();
        Motor_ApplyOutput(direction, 0.0f);
        return;
    }

    error = target_speed - actual_speed;

    /* 微分作用于测量值，改变目标转速时不会出现微分冲击。 */
    if (s_pid_measurement_ready == 0U)
    {
        derivative = 0.0f;
        s_pid_measurement_ready = 1U;
    }
    else
    {
        derivative = -(actual_speed - s_pid_last_speed) / MOTOR_CONTROL_DT;
    }
    candidate_integral = Motor_ClampFloat(
        s_pid_integral + error * MOTOR_CONTROL_DT,
        -MOTOR_PID_INTEGRAL_LIMIT,
        MOTOR_PID_INTEGRAL_LIMIT);
    feedforward = Motor_GetFeedforward(target_speed);

    output_unlimited = feedforward +
                       s_pid_kp * error +
                       s_pid_ki * candidate_integral +
                       s_pid_kd * derivative;
    output = Motor_ClampFloat(output_unlimited, 0.0f, (float)PWM_MAX);

    /* 条件积分：饱和时只允许能够把输出拉回有效范围的积分。 */
    if ((output_unlimited > 0.0f && output_unlimited < (float)PWM_MAX) ||
        (output_unlimited >= (float)PWM_MAX && error < 0.0f) ||
        (output_unlimited <= 0.0f && error > 0.0f))
    {
        s_pid_integral = candidate_integral;
    }
    else
    {
        output_unlimited = feedforward +
                           s_pid_kp * error +
                           s_pid_ki * s_pid_integral +
                           s_pid_kd * derivative;
        output = Motor_ClampFloat(output_unlimited, 0.0f, (float)PWM_MAX);
    }

    s_pid_last_speed = actual_speed;

#if (MOTOR_ENABLE_NO_ENCODER_AUTO_STOP != 0U)
    /*
     * 有足够目标和 PWM，却在整个观察窗口内几乎没有脉冲：
     * 编码器接线错误、掉线、损坏或电机堵转。锁存故障并立即断 PWM。
     */
    if (target_speed >= MOTOR_ENCODER_FAULT_MIN_TARGET_RPM &&
        output >= MOTOR_ENCODER_FAULT_MIN_DUTY)
    {
        s_encoder_watch_cycles++;
        s_encoder_watch_counts += Motor_AbsCount(encoder_count);
        if (s_encoder_watch_cycles >= MOTOR_ENCODER_FAULT_WINDOW_CYCLES)
        {
            if (s_encoder_watch_counts < MOTOR_ENCODER_FAULT_MIN_COUNTS)
            {
                Motor_LatchFault(MOTOR_CONTROL_FAULT_NO_ENCODER);
                return;
            }
            s_encoder_watch_cycles = 0UL;
            s_encoder_watch_counts = 0UL;
        }
    }
    else
    {
        s_encoder_watch_cycles = 0UL;
        s_encoder_watch_counts = 0UL;
    }
#endif

#if (MOTOR_ENABLE_WRONG_DIR_AUTO_STOP != 0U)
    /* 该项仅作可选诊断，不参与正常的转速大小闭环。 */
    if (target_speed >= MOTOR_ENCODER_FAULT_MIN_TARGET_RPM &&
        (s_actual_rpm * direction) <= -MOTOR_DIRECTION_FAULT_MIN_RPM)
    {
        s_wrong_direction_cycles++;
        if (s_wrong_direction_cycles >= MOTOR_DIRECTION_FAULT_WINDOW_CYCLES)
        {
            Motor_LatchFault(MOTOR_CONTROL_FAULT_WRONG_DIRECTION);
            return;
        }
    }
    else
    {
        s_wrong_direction_cycles = 0UL;
    }
#endif

    Motor_ApplyOutput(direction, output);
}
