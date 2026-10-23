#ifndef __CHAIN_H__
#define __CHAIN_H__

#include"NEUQ_lib_include.h"

typedef struct//用于防止零偏
{
    uint8_t editing_NullBias_Flag;
    float sum_gx;
    float sum_gy;
    float sum_gz;
    float count;
    float gx_bias;
    float gy_bias;
    float gz_bias;
}NULLBIAS_STR;

extern volatile NULLBIAS_STR NB_PREV;

// typedef struct
// {
//     uint8_t FAX;
//     uint8_t FAY;
//     uint8_t FAZ;
//     uint8_t FGX;
//     uint8_t FGY;
//     uint8_t FGZ;
// }Menu_Params;

void Menu_Task(void);
void Menu_Init(void);
void Menu_Show(void);


#endif 
 