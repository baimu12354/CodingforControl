#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_
#include "NEUQ_lib_include.h"

typedef struct { float roll, pitch, yaw; } EulerAngle_t;
extern EulerAngle_t euler;

void Attitude_Init(void);   // 上电静止校准
void Attitude_Update(void); // 每 5ms 调用

#endif