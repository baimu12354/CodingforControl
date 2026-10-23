#include "NEUQ_lib_include.h"

volatile MotorData motor=0;
volatile int32_t motor_ms;

void MOTOR_Init(){
    NEUQ_GPIO_init(PA5,GPIO_Mode_Output_PushPull,1);
    NEUQ_GPIO_init(PC7,GPIO_Mode_Output_PushPull,1);
    NEUQ_PWM_init(TIM2_CH3_PA2,8000,1000);
    NEUQ_PWM_init(TIM8_CH1_PC6,8000,1000);
    NEUQ_Encoder_init(Encoder_TIM3,Encoder_TIM3_PhaseA_PA6,Encoder_TIM3_PhaseB_PA7);  
    NEUQ_Encoder_init(Encoder_TIM4,Encoder_TIM4_PhaseA_PB6,Encoder_TIM4_PhaseB_PB7);  
    pid_parameters_init();
    NEUQ_PIT_init_ms(PIT_CH1,1);
    motor_ms=0;
    last_pick=0;
    now=0;
    motor.running=motor.target_rpm=motor.pid.Kp=motor.pid.Ki=motor.pid.Kd=0;
}
void MOTOR_Start(){
    motor.running=1;
    Ipid_parameters_setting(&motor.pid,motor.pid.Kp, motor.pid.Ki,motor.pid.Kd);
    NEUQ_Encoder_Clear(Encoder_TIM4);
}

void MOTOR_Stop(){
    motor.running=0;
    NEUQ_PWM_set_duty(TIM8_CH1_PC6,0);
    NEUQ_PWM_set_duty(TIM2_CH3_PA2, 0); 
}  

void MOTOR_Set_PID(float PID_P,float PID_I,float PID_D){
    if(motor.running==0){
        Ipid_parameters_setting(&motor.pid,PID_P, PID_I,PID_D);
    }
    else{
        return;
    }
} 

void MOTOR_Set_RPM(float RPM){
    motor.target_rpm=RPM;
} 

void MOTOR_Task(){
    motor_ms++; 
    static float duty =0;
    static float raw_rpm=0;
        raw_rpm=NEUQ_Encoder_Read(Encoder_TIM4)*125;
        NEUQ_Encoder_Clear(Encoder_TIM4);                 
        motor.current_rpm=(1-Alpha)* motor.current_rpm+ Alpha* raw_rpm;
        if (motor.running)   
            {   
                float delta = IncrementalPID_Update(&motor.pid,motor.target_rpm,motor.current_rpm);
                duty +=delta;

                if (duty>=0){
                    if (duty > 10000) duty= 10000;
                    NEUQ_GPIO_write(PA5,1);
                    NEUQ_GPIO_write(PC7,1);
                    NEUQ_PWM_set_duty(TIM8_CH1_PC6, (uint32_t)duty);
                    NEUQ_PWM_set_duty(TIM2_CH3_PA2, (uint32_t)duty);
                }else{
                    duty=-duty;
                    if (duty > 10000) duty= 10000;
                    NEUQ_GPIO_write(PA5,0);
                    NEUQ_GPIO_write(PC7,0);
                    NEUQ_PWM_set_duty(TIM8_CH1_PC6, (uint32_t)duty);
                    NEUQ_PWM_set_duty(TIM2_CH3_PA2, (uint32_t)duty);
                    duty=-duty;
                }
            } else {
                NEUQ_PWM_set_duty(TIM8_CH1_PC6,0);
                NEUQ_PWM_set_duty(TIM2_CH3_PA2,0);
            
    }   
}


