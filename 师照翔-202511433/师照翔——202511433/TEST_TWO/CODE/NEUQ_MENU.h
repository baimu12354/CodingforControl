#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include"stdint.h"
#include"NEUQ_KEY.h"
typedef enum{
    MENU_NONE=-1,
    MENU_MAIN,//主页面
    MENU_DATA_DISPLAY,//数据显示
    MENU_X_ACCEL,//x轴——加速度
    MENU_Y_ACCEL,//y轴——加速度
    MENU_Z_ACCEL,//z轴——加速度
    MENU_X_GYRO,//x轴——角速度
    MENU_Y_GYRO,//y轴——角速度
    MENU_Z_GYRO,//z轴——角速度
    MENU_WAVEFORM_SET,//波形设置与滤波
    MENU_WAVE_SENT,//波形发送选择
    MENU_WAVE_SENT_X_ACC,//发送x轴波形acc
    MENU_WAVE_SENT_Y_ACC,//发送y轴波形acc
    MENU_WAVE_SENT_Z_ACC,//发送z轴波形acc
    MENU_WAVE_SENT_X_GYR,//发送x轴波形gyr
    MENU_WAVE_SENT_Y_GYR,//发送y轴波形gyr
    MENU_WAVE_SENT_Z_GYR,//发送z轴波形gyr
    MENU_WAVEVAARIABLE_SET,//滤波参数的设置
    WAVE_SET_X_ACC,//x轴加速度滤波
    WAVE_SET_Y_ACC,//y轴加速度滤波
    WAVE_SET_Z_ACC,//z轴加速度滤波
    WAVE_SET_X_GYR,//x轴角速度滤波
    WAVE_SET_Y_GYR,//y轴角速度滤波
    WAVE_SET_Z_GYR,//z轴角速度滤波
    MENU_ATTITUDE_CALCULA,//姿态解算
    MENU_ROLL,//横滚角
    MENU_PITCH,//俯仰角
    MENU_YAW,//航向角
    MENU_COUNT
      
}MENU_ID;//菜单编号

typedef struct{
   float x_accel;
   float y_accel;
   float z_accel;
   float x_gyro;
   float y_gyro;
   float z_gyro;
}MENU_ACCANDGYR;//六轴变量结构体

typedef struct{
   float roll;
   float pitch;
   float yaw;
}MENU_CALCULA;//姿态解算结构体

//滤波参数
extern float waveset_x_acc;
extern float waveset_y_acc;
extern float waveset_z_acc;
extern float waveset_x_gyr;
extern float waveset_y_gyr;
extern float waveset_z_gyr;

typedef struct
{
   const char *name;//当前页面的名字
   MENU_ID parent_id;//父节点地址
   MENU_ID self_id;//自己节点的地址
   float *value;//当前页面参数地址
   float max;
   float min;
   float step;
}Menuconfig;//节点内容配置

//链表节点
typedef struct MENU_NODE{
const Menuconfig *info; //节点内容
struct MENU_NODE *parent;
struct MENU_NODE *child;
struct MENU_NODE *prev;
struct MENU_NODE *next;
}MENU_NODE;

extern MENU_NODE nodes[MENU_COUNT];//存储所有节点，nodes[i]就是一个节点
extern MENU_NODE *page;//当前页面的节点
extern MENU_NODE *select;//选择高亮项的节点
extern uint8_t editing;//0->浏览模式，1->编辑模式
extern float edit_temp;//用于临时存储编辑模式下更改变量的值

extern MENU_ACCANDGYR acgy_raw;
extern MENU_ACCANDGYR acandgy;
extern MENU_CALCULA attitude;

void Menu_Init(void);
uint8_t Menu_Update(KEY_EVENT_ENUM Key);
void Menu_Show(void);
void Menu_Task(void);
void Menu_Show_Updata_Data(void);

#endif