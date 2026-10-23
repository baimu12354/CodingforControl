#include "Motor.h"

Motor_t motor[MOTOR_COUNT] = {0};
uint8_t motor_enable = 0;

static const PWM_Ports_ENUM   pwm_pin[2] = {MOTOR1_PWM, MOTOR2_PWM};
static const GPIO_Ports_ENUM  dir_pin[2] = {MOTOR1_DIR, MOTOR2_DIR};
static const Encoder_DEV_ENUM enc_dev[2] = {MOTOR1_ENC, MOTOR2_ENC};

void Motor_Init(void)
{

    NEUQ_GPIO_init(PB0, GPIO_Mode_Output_PushPull, GPIO_High);

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        NEUQ_GPIO_init(dir_pin[i], GPIO_Mode_Output_PushPull, 0); 
        NEUQ_PWM_init(pwm_pin[i], MOTOR_PWM_FREQ, 0);
        motor[i].Kp = 2.0f;
        motor[i].Ki = 0.25f;
        motor[i].Kd = 0.0f;
        motor[i].integral = 0.0f;
        motor[i].last_error = 0.0f;
    }
    NEUQ_Encoder_init(MOTOR1_ENC, MOTOR1_ENC_A, MOTOR1_ENC_B);
    NEUQ_Encoder_init(MOTOR2_ENC, MOTOR2_ENC_A, MOTOR2_ENC_B);
    motor_enable = 0;
}

void Motor_SetEnable(uint8_t en)
{
    motor_enable = en;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        motor[i].integral = 0.0f;     
        motor[i].last_error = 0.0f;
        if (!en) {
            NEUQ_PWM_set_duty(pwm_pin[i], 0);
            NEUQ_GPIO_write(dir_pin[i], 0); 
            motor[i].target_rpm = 0;
        }
    }
}

void Motor_SetTarget(uint8_t idx, float rpm)
{
    if (idx >= MOTOR_COUNT) return;
    // 过零清积分
    if ((motor[idx].target_rpm * rpm <= 0.0f) || (rpm > -1.0f && rpm < 1.0f))
    {
        motor[idx].integral = 0;
        motor[idx].last_error = 0;
    }
    motor[idx].target_rpm = rpm;
}

void Motor_SetPID(float kp, float ki, float kd)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (kp >= 0) motor[i].Kp = kp;
        if (ki >= 0) motor[i].Ki = ki;
        if (kd >= 0) motor[i].Kd = kd;
        motor[i].integral = 0;       
        motor[i].last_error = 0;
    }
}

float Motor_GetTarget(uint8_t idx) { return (idx < MOTOR_COUNT) ? motor[idx].target_rpm : 0; }
float Motor_GetActual(uint8_t idx) { return (idx < MOTOR_COUNT) ? motor[idx].actual_rpm : 0; }
float Motor_GetKp(void) { return motor[0].Kp; }
float Motor_GetKi(void) { return motor[0].Ki; }
float Motor_GetKd(void) { return motor[0].Kd; }


void Motor_Control_Loop(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        // 测速
        int32_t pulse = NEUQ_Encoder_Read(enc_dev[i]);
        NEUQ_Encoder_Clear(enc_dev[i]);
        float raw_rpm = (float)pulse * 60.0f / ((float)MOTOR_ENCODER_CPR * MOTOR_CTRL_PERIOD_S);
        
        if (raw_rpm > 2000.0f || raw_rpm < -2000.0f) {
            raw_rpm = motor[i].actual_rpm; 
        }
        motor[i].actual_rpm = 0.8f * motor[i].actual_rpm + 0.2f * raw_rpm;

        // 速度过小停止
        if (!motor_enable || (motor[i].target_rpm > -20.0f && motor[i].target_rpm < 20.0f))
        {
            NEUQ_PWM_set_duty(pwm_pin[i], 0);
            NEUQ_GPIO_write(dir_pin[i], 0); 
            motor[i].integral = 0;
            motor[i].last_error = 0;
            continue;
        }

        // PID 计算
        float error = motor[i].target_rpm - motor[i].actual_rpm;
        
        // 积分分离：只有在误差小于 150 的时候，才开启积分
        if (error > -150.0f && error < 150.0f) {
            motor[i].integral += error;
        }
        
        // 积分限幅
        if (motor[i].integral > 2000.0f)  motor[i].integral = 2000.0f;
        if (motor[i].integral < -2000.0f) motor[i].integral = -2000.0f;
        
        // 计算输出
        float out = motor[i].Kp * error 
                  + motor[i].Ki * motor[i].integral 
                  + motor[i].Kd * (error - motor[i].last_error);
        motor[i].last_error = error;
        
        // 输出限幅
        if (out > PWM_MAX)       out = PWM_MAX;
        else if (out < -PWM_MAX) out = -PWM_MAX;

        // 方向与 PWM 控制
        uint32_t duty = 0;
        int32_t  scaled = (int32_t)(out * MOTOR_DIR_FIX); 

        if (scaled > 5) { 
            NEUQ_GPIO_write(dir_pin[i], 0);
            duty = (uint32_t)scaled; 
        }
        else if (scaled < -5) { 
            NEUQ_GPIO_write(dir_pin[i], 1);
            duty = (uint32_t)(-scaled); 
        }
        else { 
            NEUQ_GPIO_write(dir_pin[i], 0); 
            duty = 0; 
        }

        NEUQ_PWM_set_duty(pwm_pin[i], duty);
    }
}

void Motor_Task(void) { }