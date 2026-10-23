#ifndef OFFSET_H_
#define OFFSET_H_

#include "NEUQ_lib_include.h"

// 零偏校准状态
#define OFFSET_PENDING 0   // 未校准
#define OFFSET_RUNNING 1   // 校准中
#define OFFSET_DONE    2   // 校准完成

// 陀螺仪零偏标定
void Zero_offset(void);

#endif
