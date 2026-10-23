#include "NEUQ_lib_include.h"
#include <math.h>
IncrementalPID speed_pid = {0};
volatile float target = 0.0f;
volatile float actural = 0.0f;
volatile uint8_t motor_run = 0;
volatile uint8_t motor_stop_pending = 0;
float PWM_value(IncrementalPID *pid, float tar_speed, float cur_speed)
{
    static float pwm_value = 0.0f;
    cur_speed = Rpm_GET(Encoder_TIM3);
    actural = cur_speed;
    if (!motor_run || motor_stop_pending)
    {
        pwm_value = 0.0f;
        speed_pid.error[0] = 0.0f;
        speed_pid.error[1] = 0.0f;
        speed_pid.error[2] = 0.0f;
        motor_stop_pending = 0;
        return 0;
    }

    pwm_value += IncrementalPID_Update(pid, tar_speed, cur_speed);
    if (!isfinite(pwm_value))
    {
        pwm_value = 0.0f;

        pid->error[0] = 0.0f;
        pid->error[1] = 0.0f;
        pid->error[2] = 0.0f;

        motor_run = 0;
        motor_stop_pending = 0;

        return 0.0f;
    }
    if (pwm_value > PWM_MAX)
        pwm_value = PWM_MAX;
    if (pwm_value < 0.0f)
        pwm_value = 0.0f;
    return pwm_value;
}
void PID_UPDATE(void)
{
    NEUQ_PWM_set_duty(TIM8_CH1_PC6, (uint32_t)PWM_value(&speed_pid, target, actural));
}
