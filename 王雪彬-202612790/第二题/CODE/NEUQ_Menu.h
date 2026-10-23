#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include<stdio.h>

typedef struct{
    int32_t a_x;
    int32_t a_y;
    int32_t a_z;
    int32_t g_x;
    int32_t g_y;
    int32_t g_z;
    int32_t wave_send;    // 波形发送开关 0关 1开
    int32_t wave_axis;    // 上传轴选择 0=AX 1=AY 2=AZ 3=GX 4=GY 5=GZ
    int32_t wave_alpha_acc;   // 加速度滤波系数 0.00~1.00 
    int32_t wave_alpha_gyro;  // 角速度滤波系数 0.00~1.00 
    int32_t roll;         // 横滚角 (°) 姿态解算页面
    int32_t pitch;        // 俯仰角 (°)
    int32_t yaw;          // 航向角 (°)
}NEUQ_Menu_Params;

extern NEUQ_Menu_Params params;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);
void NEUQ_Menu_Refresh(void);

#endif