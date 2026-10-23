#include "NEUQ_lib_include.h"

void NEUQ_PID_INIT(PID_TYPEDEF *pid){
    pid->Kp = 300.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->Target = 0.0f; // 目标转速
    pid->Actual = 0.0f; // 实际转速
    pid->Error1 = 0.0f; // 本次误差
    pid->Last_Error = 0.0f; // 上次误差
    pid->Integral   = 0.0f; // 积分累加项
    pid->Output     = 0.0f; // 最终输出
    pid->Max_Integral = 3000.0f;   // 兼容旧字段（本实现按输出量程限幅，不依赖它）
    pid->Max_Output   = 10000.0f;  // 输出上限（对应 PWM 满量程）
}

float NEUQ_PID_CALC(PID_TYPEDEF *pid, float actual){
    float error;    // 本次误差
    float p_result; // 比例项
    float i_result; // 积分项
    float d_result; // 微分项

    pid->Actual = actual;
    error = pid->Target - pid->Actual;
    pid->Error1 = error;

    // 比例项
    p_result = (pid->Kp) * error;                     

    // 积分项：累加后按输出量程限幅
    if(pid->Ki == 0.0f){
        pid->Integral = 0.0f;
        i_result = 0.0f;
    }else{
        pid->Integral += error;
        i_result = (pid->Ki) * (pid->Integral);
        if(i_result > pid->Max_Output){
            i_result = pid->Max_Output;
            pid->Integral = i_result / pid->Ki;
        }else if(i_result < -pid->Max_Output){
            i_result = -pid->Max_Output;
            pid->Integral = i_result / pid->Ki;
        }
    }

    // 微分项
    d_result = (pid->Kd) * (error - pid->Last_Error); 

    pid->Output = p_result + i_result + d_result;

    // 输出限幅 + 抗积分饱和
    if(pid->Output > pid->Max_Output){
        pid->Output = pid->Max_Output;
        if(error > 0.0f){
            pid->Integral -= error;
        }
    }else if(pid->Output < -pid->Max_Output){
        pid->Output = -pid->Max_Output;
        if(error < 0.0f){
            pid->Integral += error;
        }
    }

    pid->Last_Error = error;

    return pid->Output;
}