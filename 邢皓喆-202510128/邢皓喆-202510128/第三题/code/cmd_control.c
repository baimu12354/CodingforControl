#include "NEUQ_lib_include.h"
#include <string.h>
#include <math.h>
typedef enum{
    Wait_HEADER,
    Wait_CMD,
    Wait_lENGTH,
    Wait_DATA,
    Wait_TAIL,
    Wait_CHECK
} UART_CMD_STATE;
static UART_CMD_STATE RX_state = Wait_HEADER;
static uint8_t RX_temp_CMD = 0;
static uint8_t RX_temp_DAT[12] = {0};
static uint8_t RX_temp_LENGTH = 0;
static uint8_t RX_Count = 0;
static uint8_t RX_Sum = 0;
static void cmd_control_task(uint8_t* dat, uint8_t CMD,uint8_t Length)
{
    switch (CMD)
    {
       case 0xCC:
       {
            if (Length != 1) return;
            switch (dat[0])
            {
                case 0x01:
                {
                  motor_run = 0;
                  motor_stop_pending = 1;
                    break;
                }
                case 0x02:
                {
                   motor_run = 1;
                    break;
                }
            }
            break;
        }
        case 0xDD:
        {
             ///PID参数
            float kp, ki, kd;

            if (Length != 12) return;
            if (motor_run) return;

            memcpy(&kp, dat,     4);
            memcpy(&ki, dat + 4, 4);
            memcpy(&kd, dat + 8, 4);
            if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
            kp < 0.0f || ki < 0.0f || kd < 0.0f)
            {
                return;
            }
            speed_pid.Kp = kp;
            speed_pid.Ki = ki;
            speed_pid.Kd = kd;
            break;
        }
        case 0xEE:
        {
             ///电机参数
            float new_target;

            if (Length != 4) return;
            memcpy(&new_target, dat, 4);
            if (!isfinite(new_target) || new_target < 0.0f)
            {
                return;
            }
            target = new_target;
            break;
        }
    }
}
void cmd_control_init(uint8_t dat)
{
    switch (RX_state)
    {
       case Wait_HEADER:
       {
        if(dat == UART_HEADER)
        {
            RX_state = Wait_CMD;
        }
        break;
       }
       case Wait_CMD:
       {
        RX_temp_CMD = dat;
        RX_state = Wait_lENGTH;
        break;
       }
       case Wait_lENGTH:
       {
        uint8_t expected =
        (RX_temp_CMD == 0xCC) ? 1 :
        (RX_temp_CMD == 0xDD) ? 12 :
        (RX_temp_CMD == 0xEE) ? 4 : 0;

        RX_Count = 0;
        RX_Sum = 0;
        if (expected == 0 || dat != expected)
        {
            RX_state = Wait_HEADER;
            break;
        }

        RX_temp_LENGTH = dat;
        RX_state = Wait_DATA;
        break;
       }
       case Wait_DATA:
       {
          RX_temp_DAT[RX_Count] = dat;
          RX_Count++;
          if(RX_Count == RX_temp_LENGTH)
          {
            RX_Count = 0;
            RX_state = Wait_TAIL;
          }
          RX_Sum += dat;
          break;
       }
       case Wait_TAIL:
       {
        if(dat == UART_tail)
        {
          RX_state = Wait_CHECK;
        }
        else
        {
            RX_state = (dat == UART_HEADER) ? Wait_CMD : Wait_HEADER;
            RX_Sum = 0;
        }
        break;
       }
       case Wait_CHECK:
       {
        if (dat == (uint8_t)RX_Sum)
        {
            cmd_control_task(RX_temp_DAT, RX_temp_CMD, RX_temp_LENGTH);
            RX_Sum = 0;
            RX_state = Wait_HEADER;
        }
        else
        {
            RX_state = (dat == UART_HEADER) ? Wait_CMD : Wait_HEADER;
            RX_Sum = 0;
        }
        break;
       }
       default:
            RX_state = Wait_HEADER;
            RX_Sum = 0;
            break;
    }
}
