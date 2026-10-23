#include "NEUQ_MotorCtrl.h"

/* ============================== 小工具 ============================== */

static float NEUQ_MotorCtrl_Clamp(float v, float lo, float hi)
{
    if (v > hi)
    {
        return hi;
    }
    if (v < lo)
    {
        return lo;
    }
    return v;
}

static void NEUQ_MotorCtrl_ClearWindow(NEUQ_MotorCtrlType *c)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)NEUQ_MOTOR_SPEED_WIN_MS; i++)
    {
        c->d_counts[i] = 0;
    }
    c->d_sum = 0;
    c->d_idx = 0u;
}

/* ============================== 初始化 / 复位 ============================== */

void NEUQ_MotorCtrl_Init(NEUQ_MotorCtrlType *c)
{
    c->Kp = NEUQ_MOTOR_DEF_KP;
    c->Ki = NEUQ_MOTOR_DEF_KI;
    c->Kd = NEUQ_MOTOR_DEF_KD;

    c->target_rpm = 0.0f;
    c->out_max    = NEUQ_MOTOR_OUT_MAX;
    c->int_max    = NEUQ_MOTOR_INT_MAX;
    c->win_ms     = (float)NEUQ_MOTOR_SPEED_WIN_MS;
    c->run        = 0u;

    NEUQ_MotorCtrl_Reset(c);
}

void NEUQ_MotorCtrl_Reset(NEUQ_MotorCtrlType *c)
{
    c->Ek_total = 0.0f;
    c->integral = 0.0f;
    c->d_term   = 0.0f;
    c->rpm_prev = c->out.rpm_meas;
    c->lpf_init = 0u;

    NEUQ_MotorCtrl_ClearWindow(c);

    c->out.duty        = 0.0f;
    c->out.duty_abs    = 0.0f;
    c->out.dir_forward = 1u;
    c->out.rpm_meas    = 0.0f;
    c->out.rpm_raw     = 0.0f;
}

/* ============================== 参数设置 ============================== */

void NEUQ_MotorCtrl_SetPid(NEUQ_MotorCtrlType *c, float kp, float ki, float kd)
{
#if (NEUQ_MOTOR_PID_BUMPLESS != 0)
    float   integ_out = c->integral;
    uint8_t running   = c->run;
#endif

    c->Kp = kp;
    c->Ki = ki;
    c->Kd = kd;

    /* 清掉上一拍微分状态，避免新参数下第一拍踢一脚 */
    c->d_term   = 0.0f;
    c->rpm_prev = c->out.rpm_meas;

#if (NEUQ_MOTOR_PID_BUMPLESS != 0)
    /* 运行中改参数：按新 Ki 折算 Σe，让积分项对输出的贡献保持不变 */
    if ((running != 0u) && (ki > 0.0001f))
    {
        c->Ek_total = integ_out / ki;
    }
    else
    {
        c->Ek_total = 0.0f;
    }
#else
    c->Ek_total = 0.0f;
#endif

    c->Ek_total = NEUQ_MotorCtrl_Clamp(c->Ek_total, -c->int_max, c->int_max);
    c->integral = c->Ki * c->Ek_total;
}

void NEUQ_MotorCtrl_SetTarget(NEUQ_MotorCtrlType *c, float rpm)
{
    if (rpm < 0.0f)
    {
        rpm = 0.0f;                         /* 本工程只做单向调速 */
    }
    if (rpm > NEUQ_MOTOR_RPM_LIMIT)
    {
        rpm = NEUQ_MOTOR_RPM_LIMIT;
    }

    if (c->target_rpm == rpm)
    {
        return;
    }

    c->target_rpm = rpm;
    /* 目标变了不代表转速变了：对齐上一拍实测，微分项不会虚报尖峰 */
    c->rpm_prev = c->out.rpm_meas;
}

void NEUQ_MotorCtrl_Start(NEUQ_MotorCtrlType *c)
{
    NEUQ_MotorCtrl_Reset(c);
    c->run = 1u;
}

void NEUQ_MotorCtrl_Stop(NEUQ_MotorCtrlType *c)
{
    c->run = 0u;
    NEUQ_MotorCtrl_Reset(c);
}

/* ============================== 1 ms 节拍：灌计数 ============================== */

void NEUQ_MotorCtrl_FeedTick(NEUQ_MotorCtrlType *c, int32_t delta_counts)
{
    int32_t old;

    /* 滑动窗口：新增量进环、最老的出环，d_sum 的更新是 O(1) */
    old = c->d_counts[c->d_idx];
    c->d_counts[c->d_idx] = delta_counts;
    c->d_sum += delta_counts - old;

    c->d_idx++;
    if (c->d_idx >= (uint8_t)NEUQ_MOTOR_SPEED_WIN_MS)
    {
        c->d_idx = 0u;
    }
}

/* ============================== 5 ms 控制拍 ============================== */

void NEUQ_MotorCtrl_ControlTick(NEUQ_MotorCtrlType *c)
{
    float counts;
    float rpm_raw;
    float e;
    float out;

    /* ---------- ① 测速：窗口计数 → RPM，再低通 ---------- */
    counts  = (float)c->d_sum;
    rpm_raw = (counts * NEUQ_MOTOR_ENC_DIR_SIGN * 60000.0f)
            / (NEUQ_MOTOR_COUNTS_PER_REV * c->win_ms);

    if (c->lpf_init == 0u)
    {
        c->out.rpm_meas = rpm_raw;          /* 第一个样本直接采信 */
        c->lpf_init     = 1u;
    }
    else
    {
        c->out.rpm_meas += NEUQ_MOTOR_SPEED_LPF_ALPHA * (rpm_raw - c->out.rpm_meas);
    }
    c->out.rpm_raw = rpm_raw;

    /* ---------- ② 没启动：只测速，不出力 ---------- */
    if (c->run == 0u)
    {
        c->out.duty     = 0.0f;
        c->out.duty_abs = 0.0f;
        return;
    }

    /* ---------- ③ 位置式 PID ---------- */
    e = c->target_rpm - c->out.rpm_meas;

    /* 微分项用实测变化量，避目标阶跃时 Kd 踢出尖峰 */
    c->d_term   = 0.0f - (c->Kd * (c->out.rpm_meas - c->rpm_prev));
    c->rpm_prev = c->out.rpm_meas;

    /* 条件积分：输出已经饱和且误差还想往同方向推时，不再累积 */
    if (c->Ki > 0.0001f)
    {
        float integ_now = c->Ki * c->Ek_total;
        float out_now   = (c->Kp * e) + integ_now + c->d_term;
        uint8_t allow   = 1u;

        if ((out_now > c->out_max) && (e > 0.0f))
        {
            allow = 0u;
        }
        if ((out_now < -c->out_max) && (e < 0.0f))
        {
            allow = 0u;
        }
        if (allow != 0u)
        {
            c->Ek_total += e;
            c->Ek_total  = NEUQ_MotorCtrl_Clamp(c->Ek_total, -c->int_max, c->int_max);
        }
    }

    c->integral = c->Ki * c->Ek_total;
    out         = (c->Kp * e) + c->integral + c->d_term;
    out         = NEUQ_MotorCtrl_Clamp(out, -c->out_max, c->out_max);
    if (out < 0.0f)
    {
        out = 0.0f;                         /* 只正转：负输出按 0 处理，电机滑行 */
    }

    c->out.duty        = out;
    c->out.duty_abs    = out;
    c->out.dir_forward = 1u;
}
