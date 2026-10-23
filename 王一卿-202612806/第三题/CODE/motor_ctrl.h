#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include <stdint.h>

/* =====================================================================
 * Motor speed closed-loop module (VOFA FireWater + slider text + PID)
 *
 * Hardware (DRV8701 in PWM+DIR mode):
 *   PWM : TIM3_CH1_PC6   DIR : PB12   EN : PB13 (kept HIGH after nSLEEP reset pulse)
 *   Encoder : TIM4 PB6/PB7    Comm UART : UART_0 (PA9/PA10)
 *
 * Downlink: binary frame 0xAA 0x55 CMD LEN DATA SUM (start/stop/speed/PID)
 * Uplink:   VOFA FireWater text CSV "target,actual,Kp,Ki,Kd,duty%,voltage,encoder\n"
 *
 * Usage:
 *   NEUQ_Main.c       -> MotorCtrl_Init() / MotorCtrl_Loop()
 *   NEUQ_Interrupts.c -> library callbacks route here:
 *      NEUQ_PIT_1_CALLBACK()      calls MotorCtrl_PeriodCallback()   (1 ms)
 *      TS_UART_0_RX_CALLBACK(dat) calls MotorCtrl_RxByte(dat)        (UART RX)
 * ===================================================================== */

void MotorCtrl_Init(void);            // init: UART/encoder/PWM/PID/PIT interrupt
void MotorCtrl_Loop(void);            // main loop: UART unpack + FireWater text report
void MotorCtrl_PeriodCallback(void);  // PIT 1 ms: speed + PID + PWM update
void MotorCtrl_RxByte(uint8_t dat);   // UART RX interrupt: push byte into FIFO

#endif /* __MOTOR_CTRL_H__ */
