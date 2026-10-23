#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include <stdio.h>

typedef struct{
    int32_t x_omega;  //角速度
    int32_t x_accel;  //加速度
    int32_t y_omega;
    int32_t y_accel;
    int32_t z_omega;
    int32_t z_accel;
}NEUQ_Menu_Params; //定义结构体（六轴参数）

typedef enum{
    PAGE_TYPE_MENU = 0,
    PAGE_TYPE_SIX_AXIS,
    PAGE_TYPE_WAVE_FILIER,
    PAGE_TYPE_ATTITUDE_SOLVER,
}PageType_Enum;  //枚举一级菜单选项

typedef struct{
    int32_t magic;
    int32_t select_axis;
    int32_t send_enable;
    int32_t alpha_x100;
}WaveParams;  //定义结构体（发送选择及flash魔数）

extern NEUQ_Menu_Params params; //设置外部变量，方便工程调用

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);
void NEUQ_Menu_WaveSend(void);
void Menu_Attitude_Enter(void);
void Menu_Attitude_Exit(void);
/*声明函数*/


#endif