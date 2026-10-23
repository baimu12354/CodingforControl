#ifndef _UI_H_
#define _UI_H_


#include "NEUQ_lib_include.h"
typedef struct {
    float * const MPU_alpha;
    uint8_t * const MPU_filter_select;
    float* const MPU_raw[6];
    float* const MPU_filterd[6];
}NEUQ_MANU_Param_t;
extern NEUQ_MANU_Param_t manu_param;
void NEUQ_MANU_Init(void);
void NEUQ_MANU_Task(void);
#endif
