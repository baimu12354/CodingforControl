#ifndef __NEUQ_INTERRUPTS_H__
#define __NEUQ_INTERRUPTS_H__

#include <stdint.h>

typedef struct
{
    float    target_rpm;
    float    actual_rpm;
    float    actual_rpm_raw;
    float    pwm_out;
    float    kp;
    float    ki;
    float    kd;
    uint8_t  running;
    uint32_t loop_count;
} Ctrl_Snapshot_t;

void Ctrl_Init(void);
void Ctrl_SetTarget(float rpm);
void Ctrl_SetKp(float v);
void Ctrl_SetKi(float v);
void Ctrl_SetKd(float v);
void Ctrl_SetRun(uint8_t run);
void Ctrl_GetSnapshot(Ctrl_Snapshot_t *dst);

#endif
