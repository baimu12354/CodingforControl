
#ifndef _NEUQ_MOTOR_H_
#define _NEUQ_MOTOR_H_

#include "NEUQ_lib_include.h"

#define ENCODER_PPR        13.0f    
#define ENC_SIGN           (+1.0f)
#define CONTROL_PERIOD_MS  10       
#define PWM_FREQ_HZ        8000
#define MAX_TARGET_RPM     5000.0f   
#define DEFAULT_TARGET_RPM 800.0f     /* 按下启动后自动逼近的默认目标转速 */
#define SPEED_EMA_ALPHA    0.3f

#define DEFAULT_KP         2.0f      
#define DEFAULT_KI         0.2f
#define DEFAULT_KD         0.0f     
#define PID_I_MAX          50000.0f

#define ASCII_DEBUG_CMD_ENABLE 1    /* 1=额外启用 ASCII 文本调试命令（VOFA 终端直输 pid/speed），0=关闭 */
                                    /* 不影响二进制字节包协议，考试验收仍以二进制为准 */

#define ENC_DEV           Encoder_TIM4
#define ENC_PHASE_A       Encoder_TIM4_PhaseA_PB6
#define ENC_PHASE_B       Encoder_TIM4_PhaseB_PB7

#define PWM_PORT          TIM3_CH1_PA6
#define DIR_PIN           PA7
#define DIR_FORWARD_LEVEL GPIO_High
#define DIR_REVERSE_LEVEL ((DIR_FORWARD_LEVEL == GPIO_High) ? GPIO_Low : GPIO_High)

#define FRAME_HDR1        0xAA
#define FRAME_HDR2        0x55
#define FRAME_TAIL        0xED
#define FRAME_MAX_LEN     32

#define CMD_START         0x01   
#define CMD_STOP          0x02
#define CMD_SET_TARGET    0x03
#define CMD_SET_PID       0x04
#define CMD_ECHO          0x90
 
void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

void Motor_Init(void);
void Motor_Control_ISR(void);
void Motor_OnRxByte(uint8_t dat);
void Motor_Task(void);
#endif
