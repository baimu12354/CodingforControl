/* motor_hw — 电机底层硬件: 方向/使能/PWM/编码器/ADC */

#include "motor_hw.h"   /* app_common.h → 库 + config.h */

/* 共享变量定义(声明见 app_common.h) */
volatile int32_t pwm_duty = 0;          // 当前 PWM 占空比(±PWM_MAX, 带符号), 上报调试用
volatile float   supply_voltage = 0.0f; // 供电电压(V), ADC 采样, 上报观察

static uint32_t last_volt = 0;

/* 初始化方向/使能/PWM/编码器(带滤波)/ADC */
void Motor_HW_Init(void)
{
    // 方向脚 + 使能脚
    NEUQ_GPIO_init(MOTOR_DIR, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_GPIO_init(MOTOR_EN, GPIO_Mode_Output_PushPull, GPIO_High);  // 使能必须常高
    // PWM(初始占空比 0)
    NEUQ_PWM_init(MOTOR_PWM_PORT, MOTOR_PWM_FREQ, 0);
    // 编码器
    NEUQ_Encoder_init(ENCODER_DEV, ENCODER_A, ENCODER_B);
    // 编码器输入数字滤波: 滤掉 A/B 线上的短毛刺
    TIM4->CR1   &= ~0x0001u;                            // 先停计数器(滤波配置需在禁用时改)
    TIM4->CR1    = (TIM4->CR1   & ~0x0300u) | 0x0200u;  // CKD=10 → 4分频(f_DTS=21MHz)
    TIM4->CCMR1  = (TIM4->CCMR1 & ~0xFCFCu) | 0xF0F0u;  // IC1F=IC2F=15 → 约714ns毛刺滤波
    TIM4->CR1   |= 0x0001u;                             // 重新启动计数器
    // 供电电压 ADC 采样
    adc_init(VOLT_ADC_CH, ADC_12BIT);
}

/* 按占空比输出 PWM 并设置方向(限幅 ±PWM_MAX) */
void Motor_HW_SetPwm(float out)
{
    int32_t duty = (int32_t)out;
    if (duty > PWM_MAX)  duty = PWM_MAX;
    if (duty < -PWM_MAX) duty = -PWM_MAX;

    pwm_duty = duty;                                  // 记录带符号占空比(±PWM_MAX), 供上报观察

    if (duty > 0) {                    // 正转
        NEUQ_GPIO_write(MOTOR_DIR, GPIO_High);
    } else if (duty < 0) {             // 反转
        NEUQ_GPIO_write(MOTOR_DIR, GPIO_Low);
        duty = -duty;
    }
    // duty==0 时仅占空比清零(方向不变)，电机停止
    NEUQ_PWM_set_duty(MOTOR_PWM_PORT, (uint32_t)duty);
}

/* 读取编码器计数值 */
int32_t Motor_HW_ReadEncoder(void)
{
    return NEUQ_Encoder_Read(ENCODER_DEV);
}

/* 100ms 一次采样供电电压*/
void Motor_HW_SampleVoltage(void)
{
    // 100ms 一次, ADC 均值滤波; 不在 1ms 中断里做, 避免阻塞控制
    if (time_tick - last_volt >= VOLT_SAMPLE_MS)
    {
        last_volt = time_tick;
        supply_voltage = (float)adc_mean_filter_convert(VOLT_ADC_CH, 8)
                         * VOLT_VREF / 4096.0f * VOLT_DIV_RATIO;
    }
}
