#ifndef _MPU_H_
#define _MPU_H_

#include "NEUQ_lib_include.h"

extern float raw[6];        // 原始值：0~2 陀螺(deg/s)，3~5 加速度(g)
extern float filt[6];       // 滤波后的值
extern int32_t param[6];    // 六个轴的滤波系数 α×100（1~99）

extern float roll, pitch, yaw;  // 横滚、俯仰、航向角，单位：度

void MPU_Init(void);
void MPU_Update(void);      // 中断里每 2ms 调一次
void Param_Load(void);      // 上电从 Flash 读滤波参数
void Param_Save(void);      // 把滤波参数写进 Flash

#endif