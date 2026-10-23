#ifndef MPU6050_H
#define MPU6050_H
#include "NEUQ_lib_include.h"
typedef struct {
    float alpha;    // 0.0 ~ 1.0
    float y;
    unsigned char inited;
} LPF_t;


typedef struct { 

  int16_t  *value;
  float rawdata;
  float fl_data;
  int16_t enable;
    LPF_t name;
  
}DATA;
extern DATA datatable[6];
void VOFA(void);
void DataGet(void);
void LPF_Init(void);
#endif /* MPU6050_H */
