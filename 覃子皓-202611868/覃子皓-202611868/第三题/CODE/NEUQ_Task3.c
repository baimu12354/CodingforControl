#include "NEUQ_lib_include.h"

// ========== 全局变量定义 ==========
PosPID_t g_pos_pid;                        // 位置式 PID 
volatile float g_target_rpm = 0.0f;
volatile uint8_t g_motor_running = 0;
int32_t g_pwm_output = 0;
static uint8_t upload_cnt = 0;

// ========== 位置式 PID 复位 ==========
void PosPID_Reset(PosPID_t *pid){
    pid->integ = 0.0f;
    pid->err_prev = 0.0f;
}

// ========== 位置式 PID 计算 ==========
// 返回值直接就是 PWM 输出（绝对值，符号由方向引脚决定）
float PosPID_Update(PosPID_t *pid, float target, float actual){
    float err = target - actual;

    // --- 积分项累加 ---
    pid->integ += pid->Ki * err;

    // --- 抗积分饱和：积分项本身限幅 ---
    if (pid->integ > pid->out_max) pid->integ = pid->out_max;
    if (pid->integ < pid->out_min) pid->integ = pid->out_min;

    // --- 位置式 PID 输出 ---
    float out = pid->Kp * err
              + pid->integ
              + pid->Kd * (err - pid->err_prev);

    // --- 输出限幅 ---
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    pid->err_prev = err;
    return out;
}

// ========== 初始化 ==========
void NEUQ_Task3_Init(void){
    NEUQ_Uart_init(UART_0, UART_BAUD_115200);
    NEUQ_Motor_Init();
    NEUQ_Speed_Init();

    // 位置式 PID 参数
    g_pos_pid.Kp = 8.0f;
    g_pos_pid.Ki = 0.05f;
    g_pos_pid.Kd = 0.2f;
    g_pos_pid.integ = 0.0f;
    g_pos_pid.err_prev = 0.0f;
    g_pos_pid.out_max = (float)PWM_MAX;
    g_pos_pid.out_min = -(float)PWM_MAX;

    NEUQ_Vofa_Init();
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 3);
    g_motor_running = 0;
    g_target_rpm = 0.0f;
    g_pwm_output = 0;
}

// ========== 主循环任务：解包 + 上报 ==========
void NEUQ_Task3_Task(void){

    if (++upload_cnt < 10) return;   // 10ms 上报一次
    upload_cnt = 0;

    float upload_buf[5];
    upload_buf[0] = g_target_rpm;
    upload_buf[1] = NEUQ_Speed_GetRPM();
    upload_buf[2] = g_pos_pid.Kp;
    upload_buf[3] = g_pos_pid.Ki;
    upload_buf[4] = g_pos_pid.Kd;
    NEUQ_Uart_write_data(UART_0, (uint8_t *)upload_buf, 20);

    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    NEUQ_Uart_write_data(UART_0, tail, 4);
}

// ========== 1ms 中断：测速 + 位置式 PID + PWM 更新 ==========
void NEUQ_Task3_Tick_10ms(void){
    // 静态变量：跨调用保持
    static uint8_t speed_cnt = 0;        // 测速分频计数
    static float   g_pwm_out_f = 0.0f;   // 上一拍 PWM 输出（float，用于限幅）

    // ---------- 1. 测速：每 10 次 1ms 中断更新一次（即 10ms） ----------
    if (++speed_cnt >= 10){
        speed_cnt = 0;
        NEUQ_Speed_Update();
    }

    // ---------- 2. 电机没运行：停机、复位 ----------
    if (!g_motor_running){
        NEUQ_Motor_SetOutPut(0);
        g_pwm_output = 0;
        g_pwm_out_f  = 0.0f;             // ★ 斜坡状态也清零
        PosPID_Reset(&g_pos_pid);
        return;
    }

    // ---------- 3. 位置式 PID 计算 ----------
    float actual_rpm = NEUQ_Speed_GetRPM();
    float out = PosPID_Update(&g_pos_pid, g_target_rpm, actual_rpm);

    // ---------- 4.输出限幅 ----------
    // 每 10ms 最多变化 MAX_STEP，防止瞬间饱和
    const float MAX_STEP = 5000.0f;       

    if (out - g_pwm_out_f >  MAX_STEP) out = g_pwm_out_f + MAX_STEP;
    if (out - g_pwm_out_f < -MAX_STEP) out = g_pwm_out_f - MAX_STEP;

    // 记住这一拍的输出，供下一拍用
    g_pwm_out_f = out;

    // ---------- 5. 送给电机 ----------
    g_pwm_output = (int32_t)out;
    NEUQ_Motor_SetOutPut(g_pwm_output);
}