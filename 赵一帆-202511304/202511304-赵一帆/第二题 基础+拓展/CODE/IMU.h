#ifndef _IMU_H_
#define _IMU_H_
#include "NEUQ_lib_include.h"

#define IMU_AXIS_AX 0
#define IMU_AXIS_AY 1
#define IMU_AXIS_AZ 2
#define IMU_AXIS_GX 3
#define IMU_AXIS_GY 4
#define IMU_AXIS_GZ 5
#define IMU_FLASH_ADDR 0x0807F00UL

typedef struct {
    float ax, ay, az, gx, gy, gz;
    float filtered[6];
    float alpha;
    uint8_t axis;
    uint8_t send_enable;
} IMU_Data_t;

typedef struct {
    float   alpha;
    uint8_t axis;
} IMU_Flash_t;

extern IMU_Data_t imu;
extern volatile uint8_t imu_sample_flag;

void IMU_Init(void);
void IMU_Task(void);
void IMU_SendToVOFA(void);
void IMU_EditParam(uint8_t row, int8_t dir);
void IMU_SaveParams(void);

#endif