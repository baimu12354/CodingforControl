#ifndef QUAT_H_
#define QUAT_H_

#include "NEUQ_lib_include.h"

//计算平方根倒数
float invSqrt(float a);
//IMUUpdate Mahony姿态解算
void IMU_Update(void);

#endif