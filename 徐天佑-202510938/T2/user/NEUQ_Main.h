/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_Main
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#ifndef __NEUQ_MAIN_H__
#define __NEUQ_MAIN_H__
#define KEY_UP       PE9
#define KEY_DOWN     PE8
#define KEY_ENTER    PE7
#define KEY_BACK     PE10
#define TASK2_FILTER_MIN       0.05f
#define TASK2_FILTER_MAX       0.95f

#include "../Library/NEUQ_lib/NEUQ_lib_include.h"


extern volatile uint8_t lcd_fresh ;
extern float filter_alfa_acc;
extern float filter_alfa_gyro;
typedef struct 
{ 

    float filter_alfa_acc;
    float filter_alfa_gyro;
    uint8_t axis; 
    uint8_t value;
    
} Task2Saved;
extern Task2Saved saved ;




typedef enum {
        MENU_FILTER_AX=0,
        MENU_FILTER_AY,
        MENU_FILTER_AZ,
        MENU_FILTER_GX,
        MENU_FILTER_GY,
        MENU_FILTER_GZ,
        AXIS_NONE
} AXIS;

extern volatile AXIS vofa_axis;



extern float filter_alfa;




typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float acc_x_f, acc_y_f, acc_z_f;
    float gyro_x_f, gyro_y_f, gyro_z_f;
    float roll, pitch, yaw;
} imudata;


extern volatile imudata imudata_;
// extern volatile uint8_t key_pressed[4];
// extern volatile uint8_t key_last[4];
// extern volatile AXIS vofa_axis;
// extern volatile imudata task2_imu;

// extern volatile uint8_t musave;


// void filter_reset( imudata *seed);
// float LowPass(float previous, float sample, float alpha);
// void Menu_Init(void);
// void Menu_KeyProcess(key_name key);
// void Menu_Draw(void);
// void MenuRefresh(void);
// void sendvofa(void);
// void User_Main(void);

#endif  /* __NEUQ_MAIN_H__ */
