/* motor_pid — PID算法 */

#include "motor_pid.h"   /* app_common.h → 库 + config.h */

/* 复位积分项与前次测量值 */
void PID_Reset(PID_t *p)
{
    p->integral = 0;
    p->prev_actual = 0;
}

/* 计算 PID 输出: bang-bang + 前馈 + 抗饱和 */
float PID_Compute(PID_t *p, float target, float actual)
{
    float err = target - actual;
    static uint8_t in_bang = 0;

    // bang-bang 只正向满油门加速(响应快): 减速方向不反向满油门(否则冲过头震荡), 交给 PID 负误差+微分刹车
    if (!in_bang && err > BANG_RPM) { in_bang = 1; }
    else if (in_bang && err < (BANG_RPM - BANG_HYST)) { in_bang = 0; p->integral = 0; p->prev_actual = actual; }
    if (in_bang) return PWM_MAX;

    // 速度前馈: 目标越接近最高转速, 反电动势需要的占空比越接近满值,
    // 直接按目标给基准占空比, 积分只补小误差(否则积分要从 0 攒满 ~97% 占空比, 响应极慢)
    float ff = target / MOTOR_MAX_RPM * PWM_MAX;

    // 微分项用"测量值"而非"误差": 误差微分在目标阶跃瞬间会跳变, 产生微分冲击尖峰(反超调/抖动);
    // -Kd·(actual-prev_actual) 只对实际转速的变化刹车, 目标跳变不产生尖峰
    float out = ff + p->Kp * err + p->Ki * p->integral + p->Kd * (p->prev_actual - actual);
    p->prev_actual = actual;

    // 条件积分抗饱和: 输出已被限幅且误差还在往限幅方向推时, 停止累加(防前馈+积分顶满导致超调)
    int sat = (out >= PWM_MAX && err > 0.0f) || (out <= -PWM_MAX && err < 0.0f);
    if (!sat) p->integral += err;

    // 积分限幅: 积分项最多贡献占满占空比 PWM_MAX
    float imax = PWM_MAX / (p->Ki > 0.001f ? p->Ki : 0.001f);
    if (p->integral > imax) p->integral = imax;
    if (p->integral < -imax) p->integral = -imax;

    return out;
}
