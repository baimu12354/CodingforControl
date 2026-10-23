#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include <stdio.h>


typedef struct {
    int32_t wave_axis;     // 0~5 代表6个轴 (X加速度、Y加速度、Z加速度、X陀螺、Y陀螺、Z陀螺)
    int32_t send_enable;   // 0:关 1:开
    int32_t filter_alpha;  // 滤波参数，比如 0~100（代表0.00~1.00的系数）
} NEUQ_MENU_PARAMS;

extern NEUQ_MENU_PARAMS params;

void NEUQ_MENU_INIT(void);
void NEUQ_MENU_TASK(void);

#endif 