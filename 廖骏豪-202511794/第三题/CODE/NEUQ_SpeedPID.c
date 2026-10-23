/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_SpeedPID
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
*
* 控制律：
*     out = Kp·e + Ki·Σe + Kd·(0 − Δ实测) + FF·目标
*   ① 积分按拍累加 Σe，不乘 dt，Ki 的单位就是"输出 / RPM / 拍"；
*   ② 微分用实测变化量，不除 dt，换目标那一拍不会踢出尖峰；
*   ③ 条件积分：饱和且误差还在往同方向推时才停止累加，不做反算；
*   ④ 只正转时输出限幅到 [0, out_max]，想刹车就按 0 滑行。
********************************************************************************************************************/
#include "NEUQ_Config.h"
#include "NEUQ_SpeedPID.h"

/*-------------------------------------------------------------------------------------------------------------------
* 内部小工具
*------------------------------------------------------------------------------------------------------------------*/
static float NEUQ_FClamp(float v, float lo, float hi)
{
    if (v > hi) { return hi; }
    if (v < lo) { return lo; }
    return v;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       PID 初始化
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Pid_Init(NEUQ_Pid_t *p)
{
    p->kp        = NEUQ_PID_KP_DEFAULT;
    p->ki        = NEUQ_PID_KI_DEFAULT;
    p->kd        = NEUQ_PID_KD_DEFAULT;
    p->kff       = NEUQ_PID_KFF_DEFAULT;
    p->i_limit   = NEUQ_PID_I_LIMIT_DEFAULT;    /* Σe 的限幅（量纲 = RPM·拍）            */
    p->slew_limit= NEUQ_PID_SLEW_DEFAULT;       /* 每拍输出最大变化量，<=0 关闭          */
    p->out_max   =  NEUQ_MOTOR_PWM_MAX;
#if (NEUQ_MOTOR_ALLOW_REVERSE)
    p->out_min   = -NEUQ_MOTOR_PWM_MAX;
#else
    p->out_min   =  0.0f;                       /* 只正转：输出不许为负                  */
#endif

    NEUQ_Pid_Reset(p);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       在线修改 Kp/Ki/Kd（不清积分，出力不跳变）
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Pid_SetGains(NEUQ_Pid_t *p, float kp, float ki, float kd)
{
    float integ_out = p->ki * p->ek_total;      /* 改参数前积分项对输出的贡献 */

    p->kp = (kp < 0.0f) ? 0.0f : kp;
    p->ki = (ki < 0.0f) ? 0.0f : ki;
    p->kd = (kd < 0.0f) ? 0.0f : kd;

    /* 无扰切换：让 Ki·Σe 保持不变，只换增益，出力不会掉一块再慢慢积回来 */
    if ((p->started != 0u) && (p->ki > 0.0001f))
    {
        p->ek_total = integ_out / p->ki;
        p->ek_total = NEUQ_FClamp(p->ek_total, -p->i_limit, p->i_limit);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       复位调节器
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Pid_Reset(NEUQ_Pid_t *p)
{
    p->ek_total  = 0.0f;
    p->integral  = 0.0f;
    p->d_term    = 0.0f;
    p->rpm_prev  = 0.0f;
    p->out_prev  = 0.0f;
    p->started   = 0u;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       PID 定周期运算
* 参数说明       p 控制器句柄；target 目标(RPM)；meas 实测(RPM)；dt 控制周期(s)（本律不用 dt）
* 返回参数       限幅后的输出，0 ~ +out_max（只正转时）
* 备注信息       先算各分项，再限幅，最后按只正转处理
*------------------------------------------------------------------------------------------------------------------*/
float NEUQ_Pid_Update(NEUQ_Pid_t *p, float target, float meas, float dt)
{
    float err;
    float p_term;
    float ff_term;
    float out_raw;
    float out;

    (void)dt;                                   /* 积分/微分都按"每拍"算，不需要 dt */

    err = target - meas;

    /* ---------------- ① 微分先行：用实测变化量，不除 dt ---------------- */
    if (p->started == 0u)
    {
        p->rpm_prev = meas;
        p->d_term   = 0.0f;
        p->started  = 1u;
    }
    p->d_term   = 0.0f - (p->kd * (meas - p->rpm_prev));
    p->rpm_prev = meas;

    /* ---------------- ② 比例项 + 前馈 ---------------- */
    p_term  = p->kp * err;
    ff_term = p->kff * target;

    /* ---------------- ③ 条件积分抗饱和（不做反算） ----------------
     * 只有"积分还在把输出往限幅里推"时才允许累加：
     *   · 输出已顶 +限幅、误差仍为正（想推更高）-> 不积（从源头禁止 windup）
     *   · 输出顶 +限幅、误差已变负（该回落）     -> 照常积（第一时间退出饱和）
     * 这一步正是"目标改小之后转速反而顶到最大、怎么调都回不来"的根治办法。 */
    if (p->ki > 0.0001f)
    {
        float out_now = p_term + (p->ki * p->ek_total) + p->d_term + ff_term;
        uint8_t allow = 1u;

        if ((out_now > p->out_max) && (err > 0.0f)) { allow = 0u; }
        if ((out_now < p->out_min) && (err < 0.0f)) { allow = 0u; }

        if (allow != 0u)
        {
            p->ek_total += err;
            p->ek_total  = NEUQ_FClamp(p->ek_total, -p->i_limit, p->i_limit);
        }
    }

    p->integral = p->ki * p->ek_total;

    /* ---------------- ④ 合成与限幅 ---------------- */
    out_raw = p_term + p->integral + p->d_term + ff_term;
    out     = NEUQ_FClamp(out_raw, p->out_min, p->out_max);

#if (NEUQ_MOTOR_ALLOW_REVERSE == 0)
    /* 只正转：想刹车（负输出）一律按 0 —— 电机滑行，测速只会变小，不会出现"越刹越猛" */
    if (out < 0.0f) { out = 0.0f; }
#endif

    /* ---------------- ⑤ 可选：输出变化率限幅 ---------------- */
    if (p->slew_limit > 0.0f)
    {
        out = NEUQ_FClamp(out, p->out_prev - p->slew_limit, p->out_prev + p->slew_limit);
        if (out < p->out_min) { out = p->out_min; }
        if (out > p->out_max) { out = p->out_max; }
    }

    p->out_prev = out;
    return out;
}
