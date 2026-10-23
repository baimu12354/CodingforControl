#ifndef _FLITER_H_
#define _FLITER_H_

#include"NEUQ_lib_include.h"

#define FLASH_DATA_STORE 1

typedef enum 
{
    FILTER_NONE=-1,
    FILTER_AX=0,
    FILTER_AY,
    FILTER_AZ,
    FILTER_GX,
    FILTER_GY,
    FILTER_GZ,
    FILTER_NUM
}FLITER_ENUM;

typedef struct 
{
    uint8_t F_value;
    int16_t last;
    int16_t new;
    int16_t final;
}FLITER_STR;

//三角
typedef struct
{
    float roll;
    float pitch;
    float yaw;
}Calcu_Str;

extern Calcu_Str Calcu_Value;

extern volatile FLITER_STR filters[FILTER_NUM];

extern volatile uint8_t dataWrite[6];
extern volatile uint8_t dataRead[6];


void Filter_Init(void);
void Fliter_update(void);

#endif // !_FLITER_H_
