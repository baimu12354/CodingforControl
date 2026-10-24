#ifndef __FILTER_H
#define __FILTER_H

#include "NEUQ_lib_include.h"

#define FILTER_AXIS_X 0
#define FILTER_AXIS_Y 1
#define FILTER_AXIS_Z 2
#define RAD 57.29578f

typedef struct{
    float alpha;
    float gyro_threshold;
    float acc_threshold;
    uint8_t selected_axis;
    uint8_t send_enable;
    uint16_t checksum;
}FILTER_PARAMS;

extern FILTER_PARAMS g_filter_param;
extern float g_raw_angle;
extern float g_filt_angle;
extern float filtered_acc[3];
extern float filtered_gyro[3]; 

/* 四元数姿态 */
void filter_init(void);
void filter_updata(void);

float filter_get_raw(void);
float filter_get_filter(void);
uint8_t filter_get_axis(void);
void  filter_set_axis(uint8_t axis);

/* 欧拉角输出（度） */
float filter_get_roll(void);
float filter_get_pitch(void);
float filter_get_yaw(void);

/* 陀螺零偏校准 */
void  gyro_calibrate_start(void);   /* 非阻塞：开始校准 */
void  gyro_calibrate_feed(void);    /* 每次PIT2采一次样时调用 */
uint8_t gyro_calibrate_busy(void);  /* 1=校准中 0=完成/未开始 */
uint8_t gyro_calibrated(void);      /* 是否已得到零偏 */
float gyro_get_offset(uint8_t axis);/* 0=X 1=Y 2=Z，°/s */

void filter_init(void);
void filter_updata(void);
float filter_get_raw(void);
float filter_get_filter(void);
uint8_t filter_get_axis(void);
void filter_set_axis(uint8_t axis);

#endif