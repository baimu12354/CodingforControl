#include "NEUQ_lib_include.h"

static int32_t last = 0;            // 上一次的编码器读数

void Motor_Init(void)
{
    NEUQ_GPIO_init(DIR_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);  
    NEUQ_PWM_init(PWM_PIN, 8400, 0); //最大值约为8.4khz
    NEUQ_Encoder_init(ENC_DEV, ENC_A, ENC_B);
    NEUQ_Encoder_Clear(ENC_DEV);                                   
    last = 0;
}

void Motor_Set(int32_t pwm)
{
    if (pwm > 0){                               // 正转
        NEUQ_GPIO_write(DIR_PIN, GPIO_High);
        NEUQ_PWM_set_duty(PWM_PIN, (uint32_t)pwm);
    }else if (pwm < 0){                         // 反转
        NEUQ_GPIO_write(DIR_PIN, GPIO_Low);
        NEUQ_PWM_set_duty(PWM_PIN, (uint32_t)(-pwm));
    }else{                                      // 停止
        NEUQ_GPIO_write(DIR_PIN, GPIO_Low);
        NEUQ_PWM_set_duty(PWM_PIN, 0);
    }
}

int16_t Motor_Enc(void)
{
    int32_t now = NEUQ_Encoder_Read(ENC_DEV);//// 现在计数器指着多少
    int16_t d   = (int16_t)(now - last);    //1.编码器定时器TIM4是 16 位2.now-last=这段时间的增量
    last = now;
    return d;
}