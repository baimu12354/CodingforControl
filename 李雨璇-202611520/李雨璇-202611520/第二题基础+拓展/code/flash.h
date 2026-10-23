#ifndef FLASH_H_
#define FLASH_H_

#include "NEUQ_lib_include.h"

// 数据写入Flash
void Flash_Send(float *a, float *b, float *c, float *d, float *e, float *f);
// 读取Flash中的数据
void Flash_Read(void);

#endif