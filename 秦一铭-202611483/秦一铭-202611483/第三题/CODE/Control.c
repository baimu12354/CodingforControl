#include "NEUQ_lib_include.h"

static uint8_t        tick = 0;     
static float          pwm  = 0;     // PID 累加出来的输出
static IncrementalPID pid;

void Ctrl_Tick(void)                // 1ms 定时器中断里调用
{
    int16_t pulse;

    if (++tick < 5) return;         // 1ms × 5 = 5ms 控制周期
    tick = 0;

    /* 测速：5ms 的脉冲数 × 12000 / 440 = 每分钟转数
       440 = 4 倍频 × 11 线 × 减速比 10 */
    pulse        = Motor_Enc();
    g_actual_rpm = (int16_t)-(pulse * 12000 / 440);   // 一分钟里有12000个 5ms

    /* 停止：输出 0。停止时每 5ms 重装一次参数，
       所以一启动用的就是最新参数 */
    if (g_run == 0){
        pwm = 0;
        Ipid_parameters_setting(&pid, g_pid[0] / 100.0f,   //除以100.0f，避免整数除法截断，f让运算在单精度里完成
                                      g_pid[1] / 100.0f,
                                      g_pid[2] / 100.0f);
        Motor_Set(0);
        return;
    }

    //C 会自动做整数到浮点的转换
    pwm += IncrementalPID_Update(&pid,g_target_rpm,g_actual_rpm);

    /* 限幅：正目标只允许 0~PWM_MAX，负目标只允许 -PWM_MAX~0 */
    if (g_target_rpm >= 0){
        if (pwm > PWM_MAX) pwm = PWM_MAX;
        if (pwm < 0)       pwm = 0;
    }else{
        if (pwm < -PWM_MAX) pwm = -PWM_MAX;
        if (pwm > 0)        pwm = 0;
    }
    Motor_Set((int32_t)pwm);        
}