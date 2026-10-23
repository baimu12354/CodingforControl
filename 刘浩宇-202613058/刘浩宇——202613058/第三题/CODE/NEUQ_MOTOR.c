#include "NEUQ_lib_include.h"

static PID_TYPEDEF motor_pid;
static float target_speed = 0.0f;
static float actual_speed = 0.0f;
static float pwm_output   = 0.0f;   // 当前实际输出的 PWM 量
static uint8_t running = 0;

void NEUQ_MOTOR_INIT(void){
    NEUQ_PID_INIT(&motor_pid);
    motor_pid.Max_Output   = (float)PWM_MAX;   // 输出上限 = PWM 满量程（10000），与 PWM 驱动一致
    motor_pid.Max_Integral = (float)PWM_MAX;   // 兼容旧字段，新版 PID 按输出量程限幅
    motor_pid.Target = 0.0f;                   // 目标转速先清零
    target_speed = 0.0f;
    actual_speed = 0.0f;
    pwm_output   = 0.0f;
    running = 0;
}

void NEUQ_MOTOR_Start(void){
    motor_pid.Integral   = 0.0f;
    motor_pid.Last_Error = 0.0f;
    motor_pid.Output     = 0.0f;
    motor_pid.Actual     = 0.0f;
    motor_pid.Target     = target_speed;
    running = 1;
}

void NEUQ_MOTOR_Stop(void){
    running = 0;                        // 停止标志
    NEUQ_PWM_set_duty(TIM2_CH3_PA2, 0); // PWM 占空比设为 0
    pwm_output = 0.0f;
    motor_pid.Integral = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_SetTarget(float speed){
    if(speed >  50.0f){
        speed =  50.0f;
    }
    if(speed < -50.0f){
        speed = -50.0f; 
    }
    target_speed = speed;
}

void NEUQ_MOTOR_SetKp(float Kp){
    motor_pid.Kp = Kp;
    motor_pid.Integral = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_SetKi(float Ki){
    motor_pid.Ki = Ki;
    motor_pid.Integral = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_SetKd(float Kd){
    motor_pid.Kd = Kd;
    motor_pid.Integral = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_Task(void){
    float pid_output = 0.0f;   // 输出量，正负代表方向
    float duty;                // 占空比
    uint8_t flag;           // 方向标志（正负）
    actual_speed = NEUQ_ENCODER_GETSPEED();
    if(running == 0){
        NEUQ_PWM_set_duty(TIM2_CH3_PA2, 0);   // PWM 输出 0，电机不转
        pwm_output = 0.0f;
        return;
    }
    motor_pid.Target = target_speed;
    pid_output = NEUQ_PID_CALC(&motor_pid, actual_speed);   // 正负号代表方向
    flag = (pid_output < 0.0f) ? 1 : 0;
    duty = (pid_output >= 0.0f) ? pid_output : -pid_output;
    if(flag == 1){
        NEUQ_GPIO_write(PA5, GPIO_High);   // 正转
    }else{
        NEUQ_GPIO_write(PA5, GPIO_Low);    // 反转
    }

    if(duty > (float)PWM_MAX){
        duty = (float)PWM_MAX;
    }
    pwm_output = duty;
    NEUQ_PWM_set_duty(TIM2_CH3_PA2, (uint32_t)duty);
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
