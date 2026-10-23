/*位置式 PID 控制器*/
#include "NEUQ_lib_include.h" 

/* 位置式 PID：输出 = Kp*e + Ki*Σe + Kd*(e - e_last) */
void NEUQ_PID_INIT(PID_TYPEDEF *pid){
    pid->Kp = 300.0f;   // 默认比例系数
    pid->Ki = 0.0f;     // 默认积分系数
    pid->Kd = 0.0f;     // 默认微分系数
    pid->Target = 0.0f; // 目标转速
    pid->Actual = 0.0f; // 实际转速
    pid->Error1 = 0.0f; // 本次误差
    pid->Last_Error = 0.0f; // 上次误差
    pid->Integral = 0.0f; // 积分累加项
    pid->Output = 0.0f; // 最终输出
    pid->Max_Integral = 3000.0f;   // 兼容旧字段（本实现按输出量程限幅，不依赖它）
    pid->Max_Output   = 10000.0f;  // 输出上限（对应 PWM 满量程）
}

float NEUQ_PID_CALC(PID_TYPEDEF *pid, float actual){
    float error;  // 本次误差
    float p_result; // 比例项
    float i_result; // 积分项
    float d_result; // 微分项

    //记录实际值，算本次误差：e = 目标 - 实际
    pid->Actual = actual;                 // 保存实际转速
    error = pid->Target - pid->Actual;    // 误差 = 目标 - 实际
    pid->Error1 = error;                  // 同步到结构体

    //比例项：P = Kp × e（误差越大输出越大，是最主要的控制量）
    p_result = (pid->Kp) * error;                     // 比例项

    //积分项：累加后按输出量程限幅；Ki = 0 时不累加，避免以后一开积分就冲出去
    if(pid->Ki == 0.0f){
        //清空积分历史
        pid->Integral = 0.0f;
        i_result = 0.0f;
    }else{
        pid->Integral += error;             // 累加本次误差：Σe = Σe + e
        i_result = (pid->Ki) * (pid->Integral);   // I = Ki × Σe
        // 积分限幅
        if(i_result > pid->Max_Output){
            i_result = pid->Max_Output;                 // 夹到正上限
            pid->Integral = i_result / pid->Ki;         // 反算 Σe，防止继续增长
        }else if(i_result < -pid->Max_Output){
            i_result = -pid->Max_Output;                // 夹到负下限
            pid->Integral = i_result / pid->Ki;         // 反算 Σe
        }
    }

    //微分项：D = Kd × (本次误差 - 上次误差)
    d_result = (pid->Kd) * (error - pid->Last_Error); // 微分项

    //三项求和
    pid->Output = p_result + i_result + d_result;

    //输出限幅 + 抗积分饱和
    if(pid->Output > pid->Max_Output){
        pid->Output = pid->Max_Output;      // 输出夹到正上限
        if(error > 0.0f){
            pid->Integral -= error;         // 误差还在正向推 -> 退回这次累加的积分
        }
    }else if(pid->Output < -pid->Max_Output){
        pid->Output = -pid->Max_Output;     // 输出夹到负下限
        if(error < 0.0f){
            pid->Integral += error;         // 误差还在负向推 -> 退回这次累加的积分
        }
    }

    //保存本次误差
    pid->Last_Error = error;

    return pid->Output;     // 返回输出（范围 ±Max_Output，符号 = 方向）
}

void NEUQ_PID_SETPARA(PID_TYPEDEF *pid, float kp, float ki, float kd){
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
