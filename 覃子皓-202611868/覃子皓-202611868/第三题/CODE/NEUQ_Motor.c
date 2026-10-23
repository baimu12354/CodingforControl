#include "NEUQ_lib_include.h"

void NEUQ_Motor_Init(void){
    NEUQ_PWM_init(MOTOR_PWM_PIN, MOTOR_PWM_FREQ, 0);   //pwm初始化，引脚，频率
    NEUQ_GPIO_init(MOTOR_DIR_PIN, GPIO_Mode_Output_PushPull, GPIO_Low); //方向引脚初始化，设置推挽输出，默认低电平
}
void NEUQ_Motor_SetOutPut(int32_t duty){
    if(duty > 0){  //占空比大于0，正转
        NEUQ_GPIO_write(MOTOR_DIR_PIN,GPIO_Low);  //方向引脚置低电平
        if(duty > PWM_MAX) duty = PWM_MAX;  //限幅
        NEUQ_PWM_set_duty(MOTOR_PWM_PIN,(uint32_t)duty);  //pwm设置
    }else if(duty <0){  //占空比小于0，反转
        NEUQ_GPIO_write(MOTOR_DIR_PIN,GPIO_High);  //方向引脚高电平
        int32_t abs_duty = -duty;  //取绝对值
        if(abs_duty > PWM_MAX) abs_duty = PWM_MAX;  //限幅
        NEUQ_PWM_set_duty(MOTOR_PWM_PIN,(uint32_t)abs_duty);  //pwm设置
    }else{
        NEUQ_PWM_set_duty(MOTOR_PWM_PIN,0);  //占空比为0，停止
    }
}
void NEUQ_Motor_Stop(void){  //停止函数
    NEUQ_PWM_set_duty(MOTOR_PWM_PIN,0);
}
