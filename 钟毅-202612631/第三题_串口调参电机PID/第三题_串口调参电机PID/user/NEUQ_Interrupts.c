#include "../code/NEUQ_lib/NEUQ_lib_include.h"
#include <string.h>

#define MOTOR_PWM_PORT     TIM8_CH1_PC6
#define MOTOR_PWM_FREQ     8000u
#define MOTOR_DIR_PIN      PC7
#define MOTOR_DIR_INVERT   0

#define ENC_TIM            Encoder_TIM4
#define ENC_PPR            1560.0f
#define CNT_TO_RPM         (6000.0f / ENC_PPR)
#define ENC_DIR_SIGN       (+1.0f)

#define CTRL_PERIOD_MS     10u
#define SPEED_FILTER_ALPHA 0.30f
#define TARGET_RPM_MAX     6000.0f
#define PID_KP_DEFAULT     0.5f
#define PID_KI_DEFAULT     0.2f
#define PID_KD_DEFAULT     0.0f
#define PID_DELTA_MAX      300.0f

static IncrementalPID    s_pid;
static volatile float    s_target   = 0.0f;
static volatile float    s_pwm      = 0.0f;
static volatile float    s_rpm_raw  = 0.0f;
static volatile float    s_rpm_flt  = 0.0f;
static volatile uint8_t  s_running  = 0;
static volatile uint32_t s_loop_count = 0;

extern NEUQ_FIFO_t g_rx_fifo;

static void Motor_ApplyPwm(float pwm)
{
    uint32_t duty;
    if (pwm < 0.0f)
    {
        duty = (uint32_t)(-pwm + 0.5f);
        NEUQ_GPIO_write(MOTOR_DIR_PIN, MOTOR_DIR_INVERT ? GPIO_Low : GPIO_High);
    }
    else
    {
        duty = (uint32_t)(pwm + 0.5f);
        NEUQ_GPIO_write(MOTOR_DIR_PIN, MOTOR_DIR_INVERT ? GPIO_High : GPIO_Low);
    }
    NEUQ_PWM_set_duty(MOTOR_PWM_PORT, duty);
}

static void Ctrl_ResetError(void)
{
    s_pid.error[0] = 0.0f;
    s_pid.error[1] = 0.0f;
    s_pid.error[2] = 0.0f;
}

void Ctrl_Init(void)
{
    NEUQ_GPIO_init(MOTOR_DIR_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_Encoder_init(ENC_TIM, Encoder_TIM4_PhaseA_PB6, Encoder_TIM4_PhaseB_PB7);
    NEUQ_Encoder_Clear(ENC_TIM);
    NEUQ_PWM_init(MOTOR_PWM_PORT, MOTOR_PWM_FREQ, 0);
    NEUQ_PWM_enable_output(MOTOR_PWM_PORT);
    Ipid_parameters_setting(&s_pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
    s_pid.max_delta_pwm = PID_DELTA_MAX;
    Ctrl_ResetError();
    s_target = 0.0f; s_pwm = 0.0f; s_rpm_raw = 0.0f; s_rpm_flt = 0.0f;
    s_running = 0; s_loop_count = 0;
    Motor_ApplyPwm(0.0f);
}

void Ctrl_SetTarget(float rpm)
{
    if (rpm >  TARGET_RPM_MAX) rpm =  TARGET_RPM_MAX;
    if (rpm < -TARGET_RPM_MAX) rpm = -TARGET_RPM_MAX;
    s_target = rpm;
}

void Ctrl_SetKp(float v) { s_pid.Kp = v; }
void Ctrl_SetKi(float v) { s_pid.Ki = v; }
void Ctrl_SetKd(float v) { s_pid.Kd = v; }

void Ctrl_SetRun(uint8_t run)
{
    if (run)
    {
        Ctrl_ResetError();
        s_running = 1u;
    }
    else
    {
        s_running = 0u;
        s_pwm = 0.0f;
        Motor_ApplyPwm(0.0f);
    }
}

void Ctrl_GetSnapshot(Ctrl_Snapshot_t *dst)
{
    if (dst == 0) return;
    dst->target_rpm     = s_target;
    dst->actual_rpm     = s_rpm_flt;
    dst->actual_rpm_raw = s_rpm_raw;
    dst->pwm_out        = s_pwm;
    dst->kp             = s_pid.Kp;
    dst->ki             = s_pid.Ki;
    dst->kd             = s_pid.Kd;
    dst->running        = s_running;
    dst->loop_count     = s_loop_count;
}

void NEUQ_PIT_1_CALLBACK(void)
{
    int32_t cnt;
    float delta;

    cnt = NEUQ_Encoder_Read(ENC_TIM);
    NEUQ_Encoder_Clear(ENC_TIM);
    s_rpm_raw = CNT_TO_RPM * (float)cnt * ENC_DIR_SIGN;
    s_rpm_flt += SPEED_FILTER_ALPHA * (s_rpm_raw - s_rpm_flt);
    s_loop_count++;

    if (!s_running) return;

    delta = IncrementalPID_Update(&s_pid, s_target, s_rpm_flt);
    s_pwm += delta;
    if (s_pwm >  (float)PWM_MAX) s_pwm =  (float)PWM_MAX;
    if (s_pwm < -(float)PWM_MAX) s_pwm = -(float)PWM_MAX;

    Motor_ApplyPwm(s_pwm);
}

void NEUQ_PIT_2_CALLBACK(void) {}

void TS_UART_0_RX_CALLBACK(uint8_t dat) { NEUQ_FIFO_write_byte(&g_rx_fifo, dat); }
void TS_UART_1_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_2_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_3_RX_CALLBACK(uint8_t dat) { (void)dat; }

void NEUQ_EXTI0_Callback(void)  {}
void NEUQ_EXTI1_Callback(void)  {}
void NEUQ_EXTI2_Callback(void)  {}
void NEUQ_EXTI3_Callback(void)  {}
void NEUQ_EXTI4_Callback(void)  {}
void NEUQ_EXTI5_Callback(void)  {}
void NEUQ_EXTI6_Callback(void)  {}
void NEUQ_EXTI7_Callback(void)  {}
void NEUQ_EXTI8_Callback(void)  {}
void NEUQ_EXTI9_Callback(void)  {}
void NEUQ_EXTI10_Callback(void) {}
void NEUQ_EXTI11_Callback(void) {}
void NEUQ_EXTI12_Callback(void) {}
void NEUQ_EXTI13_Callback(void) {}
void NEUQ_EXTI14_Callback(void) {}
void NEUQ_EXTI15_Callback(void) {}
