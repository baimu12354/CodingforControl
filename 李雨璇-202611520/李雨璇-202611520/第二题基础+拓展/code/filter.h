#ifndef FILTER_H_
#define FILTER_H_

#include "NEUQ_lib_include.h"

typedef struct {
    float a;           //低通滤波系数（范围0-1）
    float history;     //上一时刻滤波值
} Filter_Param;

extern Filter_Param A_X;
extern Filter_Param A_Y;
extern Filter_Param A_Z;
extern Filter_Param G_X;
extern Filter_Param G_Y;
extern Filter_Param G_Z;

extern Filter_Param *axis[6];

// 滤波初始化函数
void Filter_Init(Filter_Param *param);
void Filter_Init_all(void);

// 参数更改函数
void A_Send(uint8_t a, float b);

// 一阶低通滤波计算函数
float Filter_Data(Filter_Param *param, float new);

#endif