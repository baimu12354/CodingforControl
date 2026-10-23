#ifndef _PROTOCOL_H
#define _PROTOCOL_H
#include "NEUQ_lib_include.h"
//Э��֡��ʽ ֡ͷ+���ݳ���+����+У��λ+֡β
#define PROTO_HEAD 0xAA
#define PROTO_TAIL 0xBB
#define START 0x01
#define STOP 0x02
#define SPEED 0x03
#define KP 0x04
#define KI 0x05
#define KD 0x06
#define PID_MAX_STEP 500.0f   // max PWM change per cycle (anti-jitter)
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;    // integral accumulator
    float prev_error;  // last error for derivative
    float prev_output; // last output for rate limiting
} PositionalPID;

extern PositionalPID wheel_pid;
void wheel_pid_reset(PositionalPID *p);
float wheel_pid_update(PositionalPID *p, float setpoint, float actual);

extern volatile float rgd;
extern volatile uint8_t flag;
extern volatile uint8_t motor_control_enabled;
void Analysis_Data(uint8_t b);
void Handle_Data(void);
 

#endif