#include "NEUQ_lib_include.h"

// 上电初始化
void Motor_Init(void)
{
    NEUQ_PWM_init(MOTOR_PWM_PIN,1000,0);   // 5000Hz 占空比0
    NEUQ_GPIO_init(MOTOR_DIR1_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_GPIO_init(MOTOR_DIR2_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
}

//输出
void Motor_SetOutput(int32_t out)    
{
    uint32_t duty;

    //限幅，防止PID失控
    if (out >  PWM_MAX) out =  PWM_MAX;     
    if (out < -PWM_MAX) out = -PWM_MAX;

    //取绝对值作为占空比
    duty = (out > 0) ? (uint32_t)out : (uint32_t)(-out);   

    //正转:1高2低
    if (out > 0) {
        NEUQ_GPIO_write(MOTOR_DIR1_PIN, GPIO_High);
        NEUQ_GPIO_write(MOTOR_DIR2_PIN, GPIO_Low);
    }     
    //反转：1低2高
    else if (out < 0) {
        NEUQ_GPIO_write(MOTOR_DIR1_PIN, GPIO_Low);
        NEUQ_GPIO_write(MOTOR_DIR2_PIN, GPIO_High);
    }
    //停转：都低
    else {
        NEUQ_GPIO_write(MOTOR_DIR1_PIN, GPIO_Low);
        NEUQ_GPIO_write(MOTOR_DIR2_PIN, GPIO_Low);
    }

    NEUQ_PWM_set_duty(MOTOR_PWM_PIN, duty);
}

void NEUQ_Motor_Stop(void)
{
    NEUQ_GPIO_write(MOTOR_DIR1_PIN, GPIO_Low);
    NEUQ_GPIO_write(MOTOR_DIR2_PIN, GPIO_Low);
    NEUQ_PWM_set_duty(MOTOR_PWM_PIN, 0);
}
