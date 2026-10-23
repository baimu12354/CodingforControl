#include "NEUQ_lib_include.h"

volatile Motor_PID_Data Motor_Data;

void Motor_Inite()
{
    NEUQ_Encoder_init(ENCODER_TIMER,ENCODER_PHRASE_A_PIN,ENCODER_PHRASE_B_PIN);
    NEUQ_PWM_init(MOTOR_PWM_PIN,1000,0);
    NEUQ_GPIO_init(MOTOR_DIR_PIN,GPIO_Mode_Output_PushPull,MOTOR_DIR_FORWAR);
    Motor_Data.actual=0;
    Motor_Data.error=0;
    Motor_Data.integral=0;
    Motor_Data.PWM_Record=0;
    Motor_Data.Kd=0;
    Motor_Data.Ki=0;
    Motor_Data.Kp=0;
    Motor_Data.last_error=0;
    Motor_Data.last_last_error=0;
    Motor_Data.output=0;
    Motor_Data.run=0;
    Motor_Data.target=0;
}

void Motor_PIDoutput(void)
{
    //位置式pid，下面那个增量式的没跑起来，不过也可能是单位换算的原因
    // Motor_Data.last_last_error=Motor_Data.last_error;
    Motor_Data.last_error=Motor_Data.error;
    Motor_Data.error=Motor_Data.target-Motor_Data.actual;
    Motor_Data.integral+=Motor_Data.error;
    if(Motor_Data.integral>800000) Motor_Data.integral=800000;//i限低了高速上不去
    if(Motor_Data.integral<-800000) Motor_Data.integral=-800000;
    Motor_Data.output=Motor_Data.Kp*Motor_Data.error
        +Motor_Data.Ki*Motor_Data.integral
        +Motor_Data.Kd*(Motor_Data.error-Motor_Data.last_error);
    Motor_Data.PWM_Record=Motor_Data.output;
    if(Motor_Data.PWM_Record>10000) Motor_Data.PWM_Record=10000;
    if(Motor_Data.PWM_Record<-10000) Motor_Data.PWM_Record=-10000;
    if(Motor_Data.run==1)
    {
        Motor_SetDuty(Motor_Data.PWM_Record);
    }
    else
    {
        Motor_SetDuty(0);
    }




    
    // Motor_Data.last_last_error=Motor_Data.last_error;
    // Motor_Data.last_error=Motor_Data.error;
    // Motor_Data.error=Motor_Data.target-Motor_Data.actual;
    // Motor_Data.integral+=Motor_Data.error;
    // if(Motor_Data.integral>2000)Motor_Data.integral=2000;//?
    // if(Motor_Data.integral<-2000)Motor_Data.integral=-2000;
    // Motor_Data.output=Motor_Data.Kp*(Motor_Data.error-Motor_Data.last_error)
    //     +Motor_Data.Ki*Motor_Data.integral
    //     +Motor_Data.Kd*(Motor_Data.error-2*Motor_Data.last_error+Motor_Data.last_last_error);
    // Motor_Data.PWM_Record+=Motor_Data.output;
    // if (Motor_Data.PWM_Record>10000)Motor_Data.PWM_Record=10000;
    // if (Motor_Data.PWM_Record<-10000)Motor_Data.PWM_Record=-10000;
    // if(Motor_Data.run==1)
    // {
    //     Motor_SetDuty(Motor_Data.PWM_Record);
    // }
    // else 
    // {
    //     Motor_SetDuty(0);
    // }
    
}

void Motor_SetDuty(int16_t duty)
{
    
    
    if(duty>0)NEUQ_GPIO_write(MOTOR_DIR_PIN,MOTOR_DIR_FORWAR);

    if(duty<0)
    {
        NEUQ_GPIO_write(MOTOR_DIR_PIN,MOTOR_DIR_BACK);
        duty=-duty;
    }
    NEUQ_PWM_set_duty(MOTOR_PWM_PIN,duty);
    // NEUQ_LCD200_DisplayString(190,10,"setdutuy");
}

