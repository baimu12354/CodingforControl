#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

//此处无需过多函数，正常库即可
#include <stdio.h>

typedef struct{
    int32_t ax;
    int32_t ay;
    int32_t az;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
    int32_t wave_mode;
    int32_t filt_a;
    int32_t filt_g;
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
}NEUQ_Menu_Params;

extern NEUQ_Menu_Params params;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);

#endif