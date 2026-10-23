#include "NEUQ_lib_include.h"

/* 增量式 PID：Δu = Kp*(e - e1) + Ki*e + Kd*(e - 2*e1 + e2)，u = u_last + Δu
 * 每 10ms 调用一次，所以 Ki、Kd 的时间单位就是 10ms。
 * 输出是“本次增量”，累加进 Output 后再按输出量程限幅，天然抗积分饱和。 */
void NEUQ_PID_INIT(PID_TYPEDEF *pid){
    pid->Kp = 300.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->Target = 0.0f; // 目标转速
    pid->Actual = 0.0f; // 实际转速
    pid->Error1 = 0.0f; // 本次误差
    pid->Last_Error = 0.0f; // 上次误差 e(k-1)
    pid->Prev_Error = 0.0f; // 上上次误差 e(k-2)
    pid->Output = 0.0f; // 累加后的最终输出
    pid->Max_Output = 10000.0f;  // 输出上限（对应 PWM 满量程）
}

float NEUQ_PID_CALC(PID_TYPEDEF *pid, float actual){
    float error;   // 本次误差
    float d_inc;   // 本次输出增量

    pid->Actual = actual;
    error = pid->Target - pid->Actual;
    pid->Error1 = error;

    // 增量式 PID 核心公式
    d_inc = (pid->Kp) * (error - pid->Last_Error)
          + (pid->Ki) * error
          + (pid->Kd) * (error - 2.0f * pid->Last_Error + pid->Prev_Error);

    pid->Output += d_inc;

    // 输出限幅
    if(pid->Output > pid->Max_Output){
        pid->Output = pid->Max_Output;
    }else if(pid->Output < -pid->Max_Output){
        pid->Output = -pid->Max_Output;
    }

    // 误差移位：e2 = e1，e1 = e
    pid->Prev_Error = pid->Last_Error;
    pid->Last_Error = error;

    return pid->Output;
}

void NEUQ_PID_SETPARA(PID_TYPEDEF *pid, float kp, float ki, float kd){
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
