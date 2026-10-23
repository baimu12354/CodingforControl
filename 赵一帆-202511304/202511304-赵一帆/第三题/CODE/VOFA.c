#include "VOFA.h"
#include "Motor.h"
#include <string.h>

typedef union { float f; uint8_t b[4]; } FloatBytes_t;

/* 定义通信协议 */
static uint8_t jf_buf[5];
static uint8_t jf_index = 0;
static uint32_t jf_last_tick = 0;

void VOFA_Init(void)
{
    jf_index = 0;
    jf_last_tick = 0;
}

void VOFA_RxByte(uint8_t byte)
{
    // 超过 20ms 重置
    uint32_t now = HAL_GetTick();
    if (jf_index > 0 && (now - jf_last_tick > 20)) jf_index = 0;
    jf_last_tick = now;

    jf_buf[jf_index++] = byte;

    if (jf_index >= 5)
    {
        jf_index = 0; 
        uint8_t ch = jf_buf[0];
        
        FloatBytes_t d;
        d.b[0] = jf_buf[1];
        d.b[1] = jf_buf[2];
        d.b[2] = jf_buf[3];
        d.b[3] = jf_buf[4];
        float val = d.f;

        switch (ch)
        {
            case 0: Motor_SetTarget(0, val); break; // 目标速度0
            case 1: Motor_SetTarget(1, val); break; // 目标速度1
            case 4: if (!motor_enable) Motor_SetPID(val, -1, -1); break; // Kp
            case 5: if (!motor_enable) Motor_SetPID(-1, val, -1); break; // Ki
            case 6: if (!motor_enable) Motor_SetPID(-1, -1, val); break; // Kd
            case 7: Motor_SetEnable(val > 0.5f ? 1 : 0); break;          // 启停
            default: break;
        }
    }
}

void VOFA_Task(void)
{
    static uint32_t last_tx = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tx >= 20)
    {
        last_tx = now;
        float ch[8] = {
            Motor_GetTarget(0), Motor_GetActual(0),
            Motor_GetTarget(1), Motor_GetActual(1),
            Motor_GetKp(), Motor_GetKi(), Motor_GetKd(),
            (float)motor_enable
        };
        NEUQ_Uart_write_data(UART_0, (uint8_t*)ch, 32);
        static const uint8_t JF_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};
        NEUQ_Uart_write_data(UART_0, (uint8_t*)JF_TAIL, 4);
    }
}