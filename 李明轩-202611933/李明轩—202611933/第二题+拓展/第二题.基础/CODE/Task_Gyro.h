#ifndef __TASK_GYRO_H__
#define __TASK_GYRO_H__

#include <stdint.h>

// 轴序号
#define AXIS_AX 0
#define AXIS_AY 1
#define AXIS_AZ 2
#define AXIS_GX 3
#define AXIS_GY 4
#define AXIS_GZ 5
#define AXIS_NUM 6

// 存入 Flash 的配置结构
typedef struct{
    uint32_t magic;      // 校验标志：判断 Flash 里是不是有效配置
    float    alpha;      // 低通滤波系数 0.01~1.00，越小越平滑
    uint8_t  sel_axis;   // 当前选中的轴 0~5
    uint8_t  send_on;    // 波形发送开关 0/1
}FilterConfig_t;

// 供 Key.c（菜单/界面）访问的数据
extern FilterConfig_t cfg;
extern float raw[AXIS_NUM];
extern float yaw_angle;
extern const char Axis_Names[AXIS_NUM][3];

// 供 Key.c 在菜单确认时调用
void Cfg_Save(void);

// 任务接口
void Task_Gyro_Init(void);
void Task_Gyro_Loop(void);

#endif
