#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_

#include "NEUQ_lib_include.h"
#include <math.h>

typedef struct {
    float pitch;    // 俯仰角: 机头上下摆 [-90, 90]
    float roll;     // 横滚角: 机身左右转 [-180, 180]
    float yaw;      // 偏航角: 绕 Z 轴旋转, 会缓慢漂移 
    float q0;
    float q1;
    float q2;
    float q3;
} Attitude_t;

extern Attitude_t g_attitude;    //全局姿态结果, 菜单直接读

//调零
void Attitude_Ini(void);
void Attitude_Init(void);

// 10ms 中断:更新姿态
void Attitude_Update(void);

#endif