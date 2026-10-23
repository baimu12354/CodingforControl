#include "NEUQ_lib_include.h"

// 如果一启动就满速飞车（说明编码器计数方向和方向脚电平相反），把这里改成 1 再编译
#define MOTOR_DIR_REVERSE   1

static PID_TYPEDEF motor_pid;
static float target_speed = 0.0f;
static float actual_speed = 0.0f;
static float pwm_output   = 0.0f;   // 当前实际输出的 PWM 量
static uint8_t running = 0;

void NEUQ_MOTOR_INIT(void){
    NEUQ_PWM_init(TIM2_CH3_PA2, 8000, 0);    // PWM 8kHz，初始占空比必须是 0，否则上电瞬间电机会窜一下
    NEUQ_GPIO_init(PA5, GPIO_Mode_Output_PushPull, GPIO_Low);   // 方向脚
    NEUQ_PID_INIT(&motor_pid);
    motor_pid.Max_Output   = (float)PWM_MAX;   // 输出上限 = PWM 满量程（10000），与 PWM 驱动一致
    motor_pid.Target = 0.0f;       // 目标转速先清零
    target_speed = 0.0f;
    actual_speed = 0.0f;
    pwm_output   = 0.0f;
    running = 0;
}

void NEUQ_MOTOR_Start(void){
    // 重新启动时清掉上一次的 PID 状态（误差、输出），避免带着旧值冲出去
    motor_pid.Prev_Error   = 0.0f;
    motor_pid.Last_Error = 0.0f;
    motor_pid.Output     = 0.0f;
    motor_pid.Actual     = 0.0f;
    motor_pid.Target     = target_speed;
    running = 1;
}

void NEUQ_MOTOR_Stop(void){
    running = 0;    // 停止标志
    NEUQ_PWM_set_duty(TIM2_CH3_PA2, 0); // PWM 占空比设为 0
    pwm_output = 0.0f;
    motor_pid.Prev_Error = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_SetTarget(float speed){
    // 限一下，防止上位机误发一个天文数字直接把电机拉满
    if(speed >  50.0f){ speed =  50.0f; }
    if(speed < -50.0f){ speed = -50.0f; }
    target_speed = speed;
}

void NEUQ_MOTOR_SetPID(float Kp, float Ki, float Kd){
    NEUQ_PID_SETPARA(&motor_pid, Kp, Ki, Kd);
    motor_pid.Prev_Error   = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_SetKp(float Kp){
    motor_pid.Kp = Kp;
    motor_pid.Prev_Error = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_SetKi(float Ki){
    motor_pid.Ki = Ki;
    motor_pid.Prev_Error = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_SetKd(float Kd){
    motor_pid.Kd = Kd;
    motor_pid.Prev_Error = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_Task(void){
    float pid_output = 0.0f;   // 输出量，正负代表方向
    float duty;                // 占空比（0 ~ PWM_MAX）
    uint8_t forward;

    actual_speed = NEUQ_ENCODER_GETSPEED();

    if(running == 0){
        NEUQ_PWM_set_duty(TIM2_CH3_PA2, 0);   // PWM 输出 0，电机不转
        pwm_output = 0.0f;
        return;
    }

    motor_pid.Target = target_speed;
    pid_output = NEUQ_PID_CALC(&motor_pid, actual_speed);   // 正负号代表方向

    forward = (pid_output >= 0.0f) ? 1 : 0;
#if MOTOR_DIR_REVERSE
    forward = forward ? 0 : 1;               // 编码器方向反了时用这个开关整体取反
#endif
    duty = (pid_output >= 0.0f) ? pid_output : -pid_output;  // PWM 只能取正值，方向靠方向脚

    if(forward){
        NEUQ_GPIO_write(PA5, GPIO_High);   // 正转：方向脚输出高电平
    }else{
        NEUQ_GPIO_write(PA5, GPIO_Low);    // 反转：方向脚输出低电平
    }

    if(duty > (float)PWM_MAX){
        duty = (float)PWM_MAX;
    }
    pwm_output = duty;
    NEUQ_PWM_set_duty(TIM2_CH3_PA2, (uint32_t)
);
}

// 读取函数
float NEUQ_MOTOR_GetTarget(void){
    return target_speed;
}

float NEUQ_MOTOR_GetActual(void){
    return actual_speed;
}

float NEUQ_MOTOR_GetDuty(void){
    return pwm_output;
}

float NEUQ_MOTOR_GetKp(void){
    return motor_pid.Kp;
}

float NEUQ_MOTOR_GetKi(void){
    return motor_pid.Ki;
}

float NEUQ_MOTOR_GetKd(void){
    return motor_pid.Kd;
}

// 查询当前是否在运行
uint8_t NEUQ_MOTOR_IsRunning(void){
    return running;
}
