#include "NEUQ_lib_include.h"

float g_kp = 0.0f;
float g_ki = 44.0f;
float g_kd = 0.0f;
int   g_run = 0;

float speed_aim = 0.0f;
float speed_now = 0.0f;

int duty_now = 0;

//电机初始化
void NEUQ_Motor_Init(void){
    NEUQ_PWM_init(MOTOR1_PWM_PIN,1000,0);
    NEUQ_GPIO_init(MOTOR1_DIR_PIN,GPIO_Mode_Output_PushPull,GPIO_Low);
    NEUQ_Encoder_init(MOTOR1_ENC_TIM,MOTOR1_ENC_A_PIN,MOTOR1_ENC_B_PIN);

    Ipid_parameters_setting(&stra_speed_wheel_1,g_kp,g_ki,g_kd);
    stra_speed_wheel_1.max_delta_pwm = MOTOR_DUTY_STEP;
}

//设置电机占空比
void NEUQ_Motor_SetDuty(int duty){
    if(duty >  PWM_MAX){duty =  PWM_MAX;}
    if(duty < -PWM_MAX){duty = -PWM_MAX;}

    if(duty >= 0){
        NEUQ_PWM_set_duty(MOTOR1_PWM_PIN,duty);
        NEUQ_GPIO_write(MOTOR1_DIR_PIN,MOTOR1_DIR_FORWARD);
    }else {
        NEUQ_PWM_set_duty(MOTOR1_PWM_PIN,-duty);
        NEUQ_GPIO_write(MOTOR1_DIR_PIN,MOTOR1_DIR_BACKWARD);
    }
}

//电机控制
void NEUQ_Motor_Control(void){
    int pulse;

    //speed_now = (float)NEUQ_Encoder_Read(MOTOR1_ENC_TIM);
    pulse = NEUQ_Encoder_Read(MOTOR1_ENC_TIM);
    NEUQ_Encoder_Clear(MOTOR1_ENC_TIM);

    //转化为 r/s
    speed_now = (float)pulse * MOTOR1_ENC_SIGN * SPEED_SCALE;

    stra_speed_wheel_1.Kp = g_kp;
    stra_speed_wheel_1.Ki = g_ki;
    stra_speed_wheel_1.Kd = g_kd;

    int delat = IncrementalPID_Update(&stra_speed_wheel_1,speed_aim,speed_now);

    duty_now += delat;
    if(duty_now >  PWM_MAX){duty_now =  PWM_MAX;}
    if(duty_now < -PWM_MAX){duty_now = -PWM_MAX;}

    if(g_run == 0){
        duty_now = 0;
        NEUQ_Motor_SetDuty(0);
        return;
    }
    NEUQ_Motor_SetDuty(duty_now); 
}

// void NEUQ_Motor_Control(void){
//     NEUQ_PWM_set_duty(MOTOR1_PWM_PIN, 0);        /* PA5 恒低 */
//     NEUQ_GPIO_write(MOTOR1_DIR_PIN, GPIO_Low);   /* PC7 也拉低 ← 关键就是这行 */
//     return;
// }