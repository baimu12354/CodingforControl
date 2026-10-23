#ifndef MENU_H
#define MENU_H

#include "NEUQ_lib_include.h"
extern int8_t Menu_cnt;

//页面编号
typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0,
    //一级菜单
    MENU_Six_axis,
    MENU_Waveform,
    MENU_Attitude,
    //六轴陀螺仪
    MENU_ax,
    MENU_ay,
    MENU_az,
    MENU_gx,
    MENU_gy,
    MENU_gz,
    //波形与滤波
    MENU_Select_transmission,
    MENU_Transmission_switch,
    MENU_Kalman_q,          //过程噪声
    MENU_Kalman_r,          //测量噪声
    //姿态解算 
    MENU_roll,
    MENU_pitch,
    MENU_yaw,
    
    
    MENU_COUNT,
} Menu_ID;

//六轴原始数据
typedef struct 
{
    float MENU_ax;
    float MENU_ay;
    float MENU_az;
    float MENU_gx;
    float MENU_gy;
    float MENU_gz;
    float MENU_COUNT;
}MENU_Six_axis_params;
//姿态解算数据
typedef struct
{
    float Attitude_roll;
    float Attitude_pitch;
    float Attitude_yaw;
}Attitude_params;
//Flash
typedef struct{
    float select_axis;      
    float transmit_switch; 
    float kalman_q;
    float kalman_r;
}Config_Flash_t;


extern Attitude_params Aparams;
extern MENU_Six_axis_params params;
extern float Kalman_q_cnt; 
extern float Kalman_r_cnt;

void Menu_Init(void);
void Menu_Task(void);
float Menu_Getvalue(Menu_ID id);
void SixAxis_Refresh(void);
void Attitude_Refresh(void);
void Config_Load(void);
void Config_Save(void);





#endif