#include "NEUQ_lib_include.h"
#include "motor.h"

/* ==================== 对外变量 ==================== */
int32_t  spd_aim  = 0;   /* 目标速度：脉冲/10ms（可正可负） */
int32_t  spd_now  = 0;   /* 实际速度：脉冲/10ms（可正可负） */
int16_t  duty_now = 0;   /* 当前占空比：±10000 */

/* PID 参数（量纲：脉冲/10ms） */
float pid_kp = 80.0f;
float pid_ki = 2.0f;
float pid_kd = 0.0f;

/* ==================== 测速 ==================== */
static float speed_filtered = 0;

#define SPEED_FILTER_A   0.5f

/* 阶跃响应测量 */
volatile int32_t step_rise_ms = 0;
static int32_t   step_last_aim = 0;
static int32_t   step_t_ms     = 0;
static uint8_t   step_active   = 0;

/* 编码器累计计数（10ms 中断自动累计，停机也跑） */
volatile int32_t  enc_total_cnt = 0;

/* 编码器方向符号 */
volatile int8_t   enc_dir_sign  = ENCODER_DIR_SIGN;

/* ==================== PID ==================== */
#define PID_OUT_MAX     10000.0f
#define PID_I_OUT_MAX    8000.0f
#define PID_OUT_SLEW     1500.0f

typedef struct {
    float integral;
    float err_last;
} pid_state_t;

static pid_state_t pid_spd = {0.0f, 0.0f};

void motor_pid_reset(void) {
    pid_spd.integral = 0.0f;
    pid_spd.err_last = 0.0f;
    duty_now         = 0;
}

static float pid_i_out(void) {
    float i = pid_ki * pid_spd.integral;
    if (i >  PID_I_OUT_MAX) i =  PID_I_OUT_MAX;
    if (i < -PID_I_OUT_MAX) i = -PID_I_OUT_MAX;
    return i;
}

static float pid_calc(float target, float actual) {
    float err = target - actual;

    float out = pid_kp * err
              + pid_i_out()
              + pid_kd * (err - pid_spd.err_last);

    /* 抗积分饱和 */
    if (!((out >  PID_OUT_MAX && err > 0.0f) ||
          (out < -PID_OUT_MAX && err < 0.0f))) {
        pid_spd.integral += err;

        if (pid_ki != 0.0f) {
            float i = pid_ki * pid_spd.integral;
            if (i >  PID_I_OUT_MAX) pid_spd.integral =  PID_I_OUT_MAX / pid_ki;
            if (i < -PID_I_OUT_MAX) pid_spd.integral = -PID_I_OUT_MAX / pid_ki;
        }

        out = pid_kp * err
            + pid_i_out()
            + pid_kd * (err - pid_spd.err_last);
    }

    pid_spd.err_last = err;
    return out;
}

/* ==================== 初始化 ==================== */
void motor_init(void) {
    NEUQ_GPIO_init(MOTOR_DIR_PIN, GPIO_Mode_Output_PushPull, MOTOR_DIR_FWD);

#if (MOTOR_USE_EN == 1)
    NEUQ_GPIO_init(MOTOR_EN_PIN, GPIO_Mode_Output_PushPull, MOTOR_EN_ON);
#endif

    NEUQ_Encoder_init(ENCODER_DEV, ENCODER_PHASE_A, ENCODER_PHASE_B);
    NEUQ_Encoder_Clear(ENCODER_DEV);

    NEUQ_PWM_init(MOTOR_PWM_PIN, 1000, 0);
    NEUQ_PWM_enable_output(MOTOR_PWM_PIN);

    motor_pid_reset();
    motor_setduty(0);
}

void motor_setduty(int duty) {
    if (duty >  10000) duty =  10000;
    if (duty < -10000) duty = -10000;

    if (duty >= 0) {
        NEUQ_PWM_set_duty(MOTOR_PWM_PIN, (uint32_t)duty);
        NEUQ_GPIO_write(MOTOR_DIR_PIN, MOTOR_DIR_FWD);
    } else {
        NEUQ_PWM_set_duty(MOTOR_PWM_PIN, (uint32_t)(-duty));
        NEUQ_GPIO_write(MOTOR_DIR_PIN, MOTOR_DIR_REV);
    }
}

/* ==================== 控制周期任务（10ms 中断调用） ==================== */
void motor_ctrl(void) {
    /* ---------- 1. 采样编码器 ---------- */
    int32_t enc_cnt = NEUQ_Encoder_Read(ENCODER_DEV);
    NEUQ_Encoder_Clear(ENCODER_DEV);
    if (enc_dir_sign < 0) enc_cnt = -enc_cnt;
    enc_total_cnt += enc_cnt;

    /* ---------- 2. 测速：一阶低通滤波 ---------- */
    float raw = (float)enc_cnt;
    speed_filtered += (raw - speed_filtered) * SPEED_FILTER_A;
    spd_now = (int32_t)(speed_filtered + (speed_filtered >= 0.0f ? 0.5f : -0.5f));

    /* ---------- 3. 停机 / 目标为 0 ---------- */
    if (!sys_running || spd_aim == 0) {
        motor_pid_reset();
        motor_setduty(0);
        return;
    }

    /* ---------- 4. 阶跃响应测量 ---------- */
    if (spd_aim != step_last_aim) {
        step_last_aim = spd_aim;
        step_t_ms     = 0;
        step_rise_ms  = 0;
        step_active   = 1;
    }
    if (step_active) {
        step_t_ms += CTRL_PERIOD_MS;
        if ((spd_aim > 0 && spd_now >= (spd_aim * 9) / 10) ||
            (spd_aim < 0 && spd_now <= (spd_aim * 9) / 10)) {
            step_rise_ms = step_t_ms;
            step_active  = 0;
        } else if (step_t_ms >= 5000) {
            step_rise_ms = -1;
            step_active  = 0;
        }
    }

    /* ---------- 5. 位置式 PID ---------- */
    float out = pid_calc((float)spd_aim, (float)spd_now);

    if (out >  PID_OUT_MAX) out =  PID_OUT_MAX;
    if (out < -PID_OUT_MAX) out = -PID_OUT_MAX;

    float step = out - (float)duty_now;
    if (step >  PID_OUT_SLEW) out = (float)duty_now + PID_OUT_SLEW;
    if (step < -PID_OUT_SLEW) out = (float)duty_now - PID_OUT_SLEW;

    duty_now = (int16_t)out;
    motor_setduty(duty_now);
}