#include "NEUQ_lib_include.h"


extern PosPID_t g_pos_pid;
extern volatile float g_target_rpm;
extern volatile uint8_t g_motor_running;   

static uint8_t vofa_state = 0;
static uint8_t vofa_cmd = 0;
static uint8_t vofa_buf[4];
static uint8_t vofa_cnt = 0;

void NEUQ_Vofa_Init(void){
    vofa_state = 0;
    vofa_cmd = 0;
    vofa_cnt = 0;
}

void NEUQ_Vofa_FeedByte(uint8_t byte){
    if (vofa_state == 0){
        if (byte >= 0xA0 && byte <= 0xA3){
            vofa_cmd = byte;
            vofa_cnt = 0;
            vofa_state = 1;
        }
        // 可选：增加启动/停止命令
        else if (byte == 0xA4){    // 启动
            g_motor_running = 1;
            g_target_rpm = 0;
            PosPID_Reset(&g_pos_pid);
        }
        else if (byte == 0xA5){    // 停止
            g_motor_running = 0;
            g_target_rpm = 0;
            PosPID_Reset(&g_pos_pid);
        }
    }
    else if (vofa_state == 1){
        vofa_buf[vofa_cnt++] = byte;
        if (vofa_cnt >= 4){
            float v;
            memcpy(&v, vofa_buf, 4);
            switch (vofa_cmd){
                case 0xA0: g_pos_pid.Kp = v; break;
                case 0xA1: g_pos_pid.Ki = v; break;
                case 0xA2: g_pos_pid.Kd = v; break;
                case 0xA3:
                    if (v >  3000.0f) v =  3000.0f;   // 正向限幅
                    if (v < -3000.0f) v = -3000.0f;   // 反向限幅，允许负值才能反转
                    g_target_rpm = v;
                    break;
                default: break;
            }
            vofa_state = 0;
            vofa_cnt = 0;
        }
    }
}