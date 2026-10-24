#ifndef _NEUQ_SIX_AXIS_DATA_PROCESSING_H_
#define _NEUQ_SIX_AXIS_DATA_PROCESSING_H_

#include "NEUQ_lib_include.h"
#include "stdio.h"

#define ALPHA1000_MIN        50
#define ALPHA1000_MAX        990
#define ALPHA1000_MOREN      300
#define ALPHA_SAVE_SECTOR    NEUQ_FLASH_LAST_SECTOR_ID
#define ALPHA_SAVE_ADDRESS   (ALPHA_SAVE_SECTOR * NEUQ_FLASH_SECTOR_SIZE)

#define RAD_TO_DEG           57.2957795f     //弧度 → 度
#define DEG_TO_RAD           0.0174532925f   //度 → 弧度
#define wuca                 0.25f
#define ATTITUDE_DT          0.01f           //采样周期
#define ATTITUDE_BETA        1.0f            //加速度计修正强度：越大收敛越快、越易受抖动影响

typedef enum{
    XA,YA,ZA,
    XG,YG,ZG,
    AXIS_COUNT
}AXIS_NAME;

//存放六轴参数
typedef struct{
    float Axis_data[6];
    float Axis_lvbodata[6];
}NEUQ_SIX_AXIS_DATA;

//存放四元数
typedef struct{
    float q0;
    float q1;
    float q2;
    float q3;
}NEUQ_QUATERNION;

//存放姿态角
typedef struct{
    float Roll;
    float Pitch;
    float Yaw;
}NEUQ_ATTITUDE;

extern NEUQ_SIX_AXIS_DATA Six_axis_data;
extern NEUQ_QUATERNION Quaternion;
extern NEUQ_ATTITUDE Attitude;

void NEUQ_SIX_AXIS_DATA_Init(void);
void NEUQ_SIX_AXIS_DATA_Get(void);
void NEUQ_SIX_AXIS_DATA_NameDisplay(void);
void NEUQ_SIX_AXIS_DATA_DataDisplay(void);
void NEUQ_SIX_AXIS_DATA_SendtoVOFA(AXIS_NAME AXIS);
void NEUQ_QUATERNION_Get(void);
void NEUQ_ATTITUDE_NameDisplay(void);
void NEUQ_ATTITUDE_DataDisplay(void);


#endif
