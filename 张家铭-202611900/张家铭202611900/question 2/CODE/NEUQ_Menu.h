#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include <stdio.h>
#include <stdint.h>

typedef struct{
    int32_t send_axis;
    int32_t send_enable;
    int32_t acc_filter_k;    // 0~100 表示 0.00~1.00，加速度低通滤波系数
    int32_t gyro_filter_k;   // 0~100 表示 0.00~1.00，角速度低通滤波系数
    int32_t save_flag;
    int32_t gyro_cal;        // 置1触发陀螺零偏重标定，标定完成后自动归0
} NEUQ_Menu_Params;

extern NEUQ_Menu_Params params;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);

#endif
