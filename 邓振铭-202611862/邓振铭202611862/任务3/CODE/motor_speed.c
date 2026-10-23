#include"NEUQ_lib_include.h"
#include"motor_speed.h"
pid_value speed_pid;
float target_speed=0.0f;
float real_speed=0.0f;

MotorState_enum motor_state=motor_stop;
void motor_speed_init(void){
    pid_parameters_setting(&speed_pid,2.0f,0.1f,0.05f);
    NEUQ_PWM_init(TIM8_CH1_PC6,1000,0);
    //NEUQ_PWM_init(TIM2_CH3_PA2,1000,2000);
    NEUQ_Encoder_init(Encoder_TIM3,Encoder_TIM3_PhaseA_PA6,Encoder_TIM3_PhaseB_PA7);
    //NEUQ_Encoder_init(Encoder_TIM4,Encoder_TIM4_PhaseA_PB6,Encoder_TIM4_PhaseB_PB7);
    NEUQ_PIT_init_us(PIT_CH1,10000);
    // DRV8701E：PH 方向(PC7)、EN 接 PWM(PC6)、nSLEEP 唤醒(PB1 拉高)
    NEUQ_GPIO_init(PC7, GPIO_Mode_Output_PushPull, GPIO_Low);   // PH 方向脚
    NEUQ_GPIO_init(PB1, GPIO_Mode_Output_PushPull, GPIO_High);  // nSLEEP 拉高唤醒(低电平休眠)
}
float encoder_getspeed(void){
    int32_t pulse=NEUQ_Encoder_Read(Encoder_TIM3);
    NEUQ_Encoder_Clear(Encoder_TIM3);
    real_speed=pulse*60.0f/(/*电机单圈转速*/  880.0f    )/0.01;
    return real_speed;
}
void motor_setpwm(float duty){

if(duty>10000)duty=10000;
if(duty<-10000)duty=-10000;

if(duty>=0){                                   // 正转
    NEUQ_GPIO_write(PC7, GPIO_Low);            // PH 低 = 正转
}else{                                         // 反转
    NEUQ_GPIO_write(PC7, GPIO_High);           // PH 高 = 反转
    duty = -duty;
}
NEUQ_PWM_set_duty(TIM8_CH1_PC6,(uint32_t)duty); // EN 接 PWM，占空比调速
}
//NEUQ_TIM_PeriodElapsedCallback(){}