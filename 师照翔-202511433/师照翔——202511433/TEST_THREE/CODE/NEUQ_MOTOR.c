#include "NEUQ_lib_include.h"

// stra_speed_wheel_1 左轮，stra_speed_wheel_2 右轮
int32_t curspeed_lv = 0;
int32_t curspeed_rv = 0;
volatile float actual_speed_lv = 0.0f;    // rev/s
volatile float actual_speed_rv = 0.0f;    // rev/s
float delta_duty_lv = 0.0f;
float delta_duty_rv = 0.0f;
float duty_lv = 0.0f;
float duty_rv = 0.0f;

volatile uint8_t pid_modify_rejected = 0;

void Motor_PID_Init(void)
{
    now_data.g_target_spd = 0.0f;
    now_data.g_kp = 50.0f;
    now_data.g_ki = 10.0f;
    now_data.g_kd = 0.0f;

    duty_lv = duty_rv = 0.0f;
    delta_duty_lv = delta_duty_rv = 0.0f;
    pid_modify_rejected = 0;

    // 左侧电机控制：PWM + 方向 + 编码器
    NEUQ_PWM_init(TIM2_CH3_PA2, 1000, 0);
    NEUQ_GPIO_init(PA5, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_Encoder_init(Encoder_TIM3,
                      Encoder_TIM3_PhaseA_PA6,
                      Encoder_TIM3_PhaseB_PA7);

    // 右侧电机控制：PWM + 方向 + 编码器
    NEUQ_PWM_init(TIM8_CH1_PC6, 1000, 0);
    NEUQ_GPIO_init(PC7, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_Encoder_init(Encoder_TIM4,
                      Encoder_TIM4_PhaseA_PB6,
                      Encoder_TIM4_PhaseB_PB7);

    Ipid_parameters_setting(&stra_speed_wheel_1,
                            now_data.g_kp,
                            now_data.g_ki,
                            now_data.g_kd);
    Ipid_parameters_setting(&stra_speed_wheel_2,
                            now_data.g_kp,
                            now_data.g_ki,
                            now_data.g_kd);
}

// 编码器计数 -> 输出轴转速 rev/s
float Encoder_Counts_To_Speed(int32_t counts)
{
    float revs = (float)counts / (ENCODER_PPR * ENCODER_MULTIPLIER * GEAR_RAT);
    return revs * (1000.0f / PIT1_PERIOD_MS);
}

/* ======================= 左右轮 PID ======================= */

// 右侧电机增量式 PID 控制
void Motor_Contral_RX(void)
{
    float target = now_data.g_target_spd;   // rev/s，带符号
    float actual = actual_speed_rv;         // rev/s，带符号
    float next_duty;
    uint32_t pwm_output;
    float mag;

    delta_duty_rv = IncrementalPID_Update(&stra_speed_wheel_2, target, actual);

    next_duty = duty_rv + delta_duty_rv;

    // 抗积分饱和：饱和时丢弃继续推向饱和的增量
    if (next_duty > PWM_MAX) {
        if (delta_duty_rv > 0.0f) delta_duty_rv = 0.0f;
        next_duty = PWM_MAX;
    } else if (next_duty < -PWM_MAX) {
        if (delta_duty_rv < 0.0f) delta_duty_rv = 0.0f;
        next_duty = -PWM_MAX;
    }
    duty_rv = next_duty;

    // 方向 + 死区 + 限幅
    if (duty_rv >= 0.0f) {
        NEUQ_GPIO_write(PC7, GPIO_High);    // 右轮正转：PC7 高电平
        mag = duty_rv;
    } else {
        NEUQ_GPIO_write(PC7, GPIO_Low);     // 右轮反转：PC7 低电平
        mag = -duty_rv;
    }

    if (mag > 0.01f) {
        pwm_output = (uint32_t)(mag + MIN_DUTY);
    } else {
        pwm_output = 0;
    }
    if (pwm_output > (uint32_t)PWM_MAX) pwm_output = (uint32_t)PWM_MAX;

    NEUQ_PWM_set_duty(TIM8_CH1_PC6, pwm_output);
}

// 左侧电机增量式 PID 控制
void Motor_Contral_LX(void)
{
    float target = now_data.g_target_spd;   // rev/s，带符号
    float actual = actual_speed_lv;         // rev/s，带符号
    float next_duty;
    uint32_t pwm_output;
    float mag;

    delta_duty_lv = IncrementalPID_Update(&stra_speed_wheel_1, target, actual);

    next_duty = duty_lv + delta_duty_lv;

    if (next_duty > PWM_MAX) {
        if (delta_duty_lv > 0.0f) delta_duty_lv = 0.0f;
        next_duty = PWM_MAX;
    } else if (next_duty < -PWM_MAX) {
        if (delta_duty_lv < 0.0f) delta_duty_lv = 0.0f;
        next_duty = -PWM_MAX;
    }
    duty_lv = next_duty;

    // 方向 + 死区 + 限幅
    if (duty_lv >= 0.0f) {
        NEUQ_GPIO_write(PA5, GPIO_Low);     // 左轮正转：PA5 低电平
        mag = duty_lv;
    } else {
        NEUQ_GPIO_write(PA5, GPIO_High);    // 左轮反转：PA5 高电平
        mag = -duty_lv;
    }

    if (mag > 0.01f) {
        pwm_output = (uint32_t)(mag + MIN_DUTY);
    } else {
        pwm_output = 0;
    }
    if (pwm_output > (uint32_t)PWM_MAX) pwm_output = (uint32_t)PWM_MAX;

    NEUQ_PWM_set_duty(TIM2_CH3_PA2, pwm_output);
}