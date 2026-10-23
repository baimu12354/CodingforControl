#include"NEUQ_lib_include.h"
#include<string.h>
#include"motor.h"

motor_t g_motor;
volatile uint32_t g_motor_tick_ms = 0;   // 10ms 节拍，Motor_ControlTick 中累加
void Motor_Init(void){
    NEUQ_PWM_init(MOTOR_PWM_CH,1000,0);
    NEUQ_GPIO_init(MOTOR_DIR_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_Encoder_init(MOTOR_ENC_TIM, MOTOR_ENC_A, MOTOR_ENC_B);

    g_motor.running      = 0;
    g_motor.target_speed = 0.0f;
    g_motor.actual_speed = 0.0f;
    g_motor.pid_output   = 0.0f;

    g_motor.kp           = 8.0f;
    g_motor.ki           = 0.5f;
    g_motor.kd           = 0.1f;

    g_motor.e0           = 0.0f;
    g_motor.e1           = 0.0f;
    g_motor.e2           = 0.0f;

    NEUQ_PWM_set_duty(MOTOR_PWM_CH, 0);
}


// 增量式 PID：Δu = Kp·(e0 - e1) + Ki·e0 + Kd·(e0 - 2e1 + e2)
float PID_cal(float target,float actual){
    g_motor.e0 = target - actual;
    float delta = g_motor.kp * (g_motor.e0 - g_motor.e1)
                + g_motor.ki * g_motor.e0
                + g_motor.kd * (g_motor.e0 - 2.0f*g_motor.e1 + g_motor.e2);
    g_motor.e2 = g_motor.e1;
    g_motor.e1 = g_motor.e0;
    return delta;
}


// 10ms 定周期中断：测速 + 控制计算 + PWM 更新
void Motor_ControlTick(void){
    g_motor_tick_ms += MOTOR_CTRL_PERIOD_MS;                 // 回传节拍计数

    int32_t counts = NEUQ_Encoder_Read(MOTOR_ENC_TIM);
    g_motor.actual_speed = Motor_CountToRPM(counts);          // 编码器计数 -> 转速(rpm)
    NEUQ_Encoder_Clear(MOTOR_ENC_TIM);

    if(!g_motor.running){
        g_motor.e0 = 0.0f;
        g_motor.e1 = 0.0f;
        g_motor.e2 = 0.0f;
        g_motor.pid_output = 0.0f;
        NEUQ_PWM_set_duty(MOTOR_PWM_CH, 0);
        return;
    }

    float delta = PID_cal(g_motor.target_speed, g_motor.actual_speed);
    g_motor.pid_output += delta;                              // 增量式积分累加

    if (g_motor.pid_output >  PID_OUT_MAX) g_motor.pid_output =  PID_OUT_MAX;
    if (g_motor.pid_output < -PID_OUT_MAX) g_motor.pid_output = -PID_OUT_MAX;

    if (g_motor.pid_output >= 0.0f) {
        NEUQ_GPIO_write(MOTOR_DIR_PIN, GPIO_High);
        NEUQ_PWM_set_duty(MOTOR_PWM_CH, (uint32_t)g_motor.pid_output);
    } else {
        NEUQ_GPIO_write(MOTOR_DIR_PIN, GPIO_Low);
        NEUQ_PWM_set_duty(MOTOR_PWM_CH, (uint32_t)(-g_motor.pid_output));
    }
}

// 目标速度(rpm) 运行中可改
void Motor_SetTargetSpeed(float speed)
{
    g_motor.target_speed = speed;
    g_motor.e0           = 0.0f;
    g_motor.e1           = 0.0f;
    g_motor.e2           = 0.0f;
}
// 停止时生效
void Motor_SetPid(float kp, float ki, float kd)
{
    if (kp < 0.0f)   kp = 0.0f;
    if (ki < 0.0f)   ki = 0.0f;
    if (kd < 0.0f)   kd = 0.0f;
    if (kp > 500.0f) kp = 500.0f;
    if (ki > 50.0f)  ki = 50.0f;
    if (kd > 50.0f)  kd = 50.0f;

    g_motor.kp = kp;
    g_motor.ki = ki;
    g_motor.kd = kd;
}
//启动电机
void Motor_Start(void)
{
    g_motor.e0           = 0.0f;
    g_motor.e1           = 0.0f;
    g_motor.e2           = 0.0f;
    g_motor.pid_output   = 0.0f;
    g_motor.running      = 1;
}
//停止电机
void Motor_Stop(void)
{
    g_motor.running      = 0;
    g_motor.pid_output   = 0.0f;
    g_motor.e0           = 0.0f;
    g_motor.e1           = 0.0f;
    g_motor.e2           = 0.0f;
    NEUQ_PWM_set_duty(MOTOR_PWM_CH, 0);
}

// 编码器计数 -> 转速(rpm, 输出轴)
// 计数为控制周期内的增量，4 倍频下每圈计数 = MOTOR_ENC_PPR * MOTOR_ENC_MULT
// rpm = counts/s × 60 / 每圈计数
float Motor_CountToRPM(int32_t counts){
    float counts_per_s = (float)counts * (1000.0f / MOTOR_CTRL_PERIOD_MS);
    return counts_per_s * 60.0f / (float)(MOTOR_ENC_PPR * MOTOR_ENC_MULT);
}
