#include "NEUQ_lib_include.h"

NEUQ_Motor_params params;

//电机控制初始化
void NEUQ_Motor_con_init(void){
    params.set_spd = 0;
    params.kp   = 150.0f;
    params.ki   = 10.0f;
    params.kd   = 300.0f;
    params.Max_I = 1000;
    params.les_spd_total = 0;
    params.les_spd_last  = 0;
    params.motor_run = 0;
}

//电机pid控制
void NEUQ_Motor_con(void){
    params.les_spd = params.set_spd - params.speed;
    if(params.motor_run == 0){              
        params.motor_duty = 0;
        params.les_spd_total = 0;
        params.les_spd_last  = 0;
        return;
    }

    params.les_spd_total += params.les_spd;
    params.les_spd_total = fmaxf(fminf(params.les_spd_total, params.Max_I), -params.Max_I);

//（应该是）位置式pid
    params.motor_duty = params.kp * params.les_spd
                      + params.ki * params.les_spd_total
                      - params.kd * (params.speed - params.speed_last);

params.speed_last = params.speed;

//限幅
    if (params.motor_duty >  10000) params.motor_duty =  10000;
    if (params.motor_duty < -10000) params.motor_duty = -10000;
    }            

