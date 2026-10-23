#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include <stdio.h>

//菜单里可编辑的运行参数
typedef struct {
    int32_t send_axis;    //发送轴选择 0~6（0=不发送，1=Ax 2=Ay 3=Az 4=Gx 5=Gy 6=Gz）
    int32_t send_enable;  //波形发送开关 0~1（0=停止上传，1=开始上传）
    int32_t filter_coef;  //低通滤波系数 0~100（0=不滤波，越大平滑越强；确认后写入 Flash）
}NEUQ_Menu_Params;

//全局参数实例
extern NEUQ_Menu_Params params;

//页面编号
typedef enum{
    MENU_NONE = -1,
    MENU_MAIN = 0, //主菜单 功能选择页
    MENU_DISPLAY,  //六轴数据显示
    MENU_AX,       //加速度X轴
    MENU_AY,       //加速度Y轴
    MENU_AZ,       //加速度Z轴
    MENU_GX,       //角速度X轴
    MENU_GY,       //角速度Y轴
    MENU_GZ,       //角速度Z轴
    MENU_WAVE,     //波形与滤波设置
    MENU_AXIS,     //发送轴选择
    MENU_SEND,     //波形发送开关
    MENU_FILTER,   //滤波参数
    MENU_ATTITUDE, //姿态解算
    MENU_ROLL,     //横滚角
    MENU_PITCH,    //俯仰角
    MENU_YAW,      //偏航角
    MENU_COUNT
} MenuID;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);
MenuID NEUQ_Menu_GetPage(void);

#endif // !NEUQ