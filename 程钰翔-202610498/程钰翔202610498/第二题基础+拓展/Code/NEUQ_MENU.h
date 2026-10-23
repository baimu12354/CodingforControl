#ifndef     _NEUQ_MENU_H_
#define     _NEUQ_MENU_H_

#include  "NEUQ_lib_include.h"


#define PAGE_MAIN   0    //主菜单
#define PAGE_NEXT   1    //第二页,其实不太确定是放在头文件合适还是原文件

extern uint8_t current_page;      //初始化当前菜单是主菜单
extern uint8_t current_choice;           //初始化当前选项是0选项，我规定的三个选项是0，1，2
extern uint8_t current_choice_new;
extern uint8_t param_change_choice;       //二级菜单中修改参数，0时无法修改，点击按键in变为1，可以修改
extern float target_coefficient;       //为啥加f，这是滤波系数,初始值定为0.2       如果无法编译就加extern在.h里
extern float gyro_x_filtered;             //x轴滤波后的值
extern float gyro_y_filtered;             //y轴滤波后的值
extern float gyro_z_filtered;             //z轴滤波后的值
extern float gyro_x_real;                 //x轴原始数据
extern float gyro_y_real;                 //y轴原始数据
extern float gyro_z_real;                 //z轴原始数据
extern uint8_t target_send_state;         //传送状态 0为停止发送 1为开始发送
extern uint8_t target_axis;               //0为x轴  1为y轴  2为x轴
  
//void NEUQ_LCD200_DisplayString(void);
void Menu_Display(void);
void MENU_Act(void);
float LowPassFilter(float input,float output,float coefficient);
void MPU_Data_Update(void);
void LowPassFilter_Update(void);
void Target_Send(void);



#endif