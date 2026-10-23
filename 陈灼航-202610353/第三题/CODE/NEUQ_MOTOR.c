/* 直流电机闭环控制模块*/
#include "NEUQ_lib_include.h"  

#define MOTOR_DIR_REVERSE   1

static PID_TYPEDEF motor_pid;        // PID 控制器实例
static float target_speed = 0.0f;    // 目标转速
static float actual_speed = 0.0f;    // 实际转速
static float pwm_output   = 0.0f;    // 当前实际输出的 PWM 量
static uint8_t running = 0;          // 运行标志 0 = 停止 1 = 运行

void NEUQ_MOTOR_INIT(void){
    NEUQ_PWM_init(TIM2_CH3_PA2, 8000, 0);    // PWM初始占空比是 0
    NEUQ_GPIO_init(PA5, GPIO_Mode_Output_PushPull, GPIO_Low);   // 方向脚
    NEUQ_PID_INIT(&motor_pid);               // PID 结构体清零并给默认参数
    motor_pid.Max_Output   = (float)PWM_MAX;   // 输出上限
    motor_pid.Max_Integral = (float)PWM_MAX;   // 兼容旧字段，新版 PID 按输出量程限幅
    motor_pid.Target = 0.0f;       // 目标转速先清零
    target_speed = 0.0f;           // 清目标
    actual_speed = 0.0f;           // 清实际
    pwm_output   = 0.0f;           // 清输出记录
    running = 0;                   // 初始为停止状态
}

void NEUQ_MOTOR_Start(void){
    motor_pid.Integral   = 0.0f;   // 清积分
    motor_pid.Last_Error = 0.0f;   // 清上次误差
    motor_pid.Output     = 0.0f;   // 清输出
    motor_pid.Actual     = 0.0f;   // 清实际值
    motor_pid.Target     = target_speed;   // 把当前目标同步给 PID
    running = 1;                   // 置运行标志，10ms 中断里开始输出
}

void NEUQ_MOTOR_Stop(void){
    running = 0;    
    NEUQ_PWM_set_duty(TIM2_CH3_PA2, 0); // PWM 占空比设为 0
    pwm_output = 0.0f;                  // 同步输出记录
    motor_pid.Integral = 0.0f;          // 清积分
    motor_pid.Last_Error = 0.0f;        // 清误差历史
}

void NEUQ_MOTOR_SetTarget(float speed){
    if(speed >  50.0f){ speed =  50.0f; }   // 目标上限 +50 转/秒
    if(speed < -50.0f){ speed = -50.0f; }   // 目标下限 -50 转/秒
    target_speed = speed;                   // 保存目标
}

void NEUQ_MOTOR_SetPID(float Kp, float Ki, float Kd){
    NEUQ_PID_SETPARA(&motor_pid, Kp, Ki, Kd);   // 一次性写入三个系数
    motor_pid.Integral   = 0.0f;   // 换参数清积分
    motor_pid.Last_Error = 0.0f;   // 清误差历史
}

void NEUQ_MOTOR_SetKp(float Kp){
    motor_pid.Kp = Kp;             // 只改 Kp
    motor_pid.Integral = 0.0f;     
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_SetKi(float Ki){
    motor_pid.Ki = Ki;             // 只改 Ki
    motor_pid.Integral = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_SetKd(float Kd){
    motor_pid.Kd = Kd;             // 只改 Kd
    motor_pid.Integral = 0.0f;
    motor_pid.Last_Error = 0.0f;
}

void NEUQ_MOTOR_Task(void){
    float pid_output = 0.0f;   // 输出量，正负代表方向
    float duty;                // 占空比（0 ~ PWM_MAX）
    uint8_t forward;           // 方向 1 = 正转 0 = 反转

    //测速
    actual_speed = NEUQ_ENCODER_GETSPEED();

    //停止状态
    if(running == 0){
        NEUQ_PWM_set_duty(TIM2_CH3_PA2, 0);   // PWM 输出 0，电机不转
        pwm_output = 0.0f;                    // 同步输出记录
        return;
    }

    //把最新目标同步给 PID，执行一次位置式 PID 计算
    motor_pid.Target = target_speed;
    pid_output = NEUQ_PID_CALC(&motor_pid, actual_speed);  
    //pid_output = 3000.0f;

    //输出分解
    forward = (pid_output >= 0.0f) ? 1 : 0;  
#if MOTOR_DIR_REVERSE
    forward = forward ? 0 : 1;               // 编码器方向反了时用这个开关整体取反
#endif
    duty = (pid_output >= 0.0f) ? pid_output : -pid_output;  // PWM 只能取正值，方向靠方向脚

    //方向脚输出
    if(forward){
        NEUQ_GPIO_write(PA5, GPIO_High);   // 正转：方向脚输出高电平
    }else{
        NEUQ_GPIO_write(PA5, GPIO_Low);    // 反转：方向脚输出低电平
    }

    //占空比再夹一次（双保险），然后写入 PWM
    if(duty > (float)PWM_MAX){
        duty = (float)PWM_MAX;
    }
    pwm_output = duty;                                // 记录当前输出
    NEUQ_PWM_set_duty(TIM2_CH3_PA2, (uint32_t)duty);  // 写入 PWM 比较寄存器
}

//读取函数
float NEUQ_MOTOR_GetTarget(void){
    return target_speed;   // 目标转速
}

float NEUQ_MOTOR_GetActual(void){
    return actual_speed;   // 实际转速
}

float NEUQ_MOTOR_GetDuty(void){
    return pwm_output;     // 当前 PWM 输出量（0~10000）
}

float NEUQ_MOTOR_GetKp(void){
    return motor_pid.Kp;   // 当前 Kp
}

float NEUQ_MOTOR_GetKi(void){
    return motor_pid.Ki;   // 当前 Ki
}

float NEUQ_MOTOR_GetKd(void){
    return motor_pid.Kd;   // 当前 Kd
}

//查询当前是否在运行
uint8_t NEUQ_MOTOR_IsRunning(void){
    return running;        // 1 = 运行，0 = 停止
}
