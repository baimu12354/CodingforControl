/* imu_filter.h - 滤波接口 */

#ifndef _IMU_FILTER_H_
#define _IMU_FILTER_H_

#include "stm32f4xx_hal.h"
#include "imu_mpu6050.h"

void  ImuFilter_Init(void);
void  ImuFilter_ResetState(void);                                   /* 只清滤波状态，保留系数 */

/* 用一帧原始采样更新六个轴，结果写入 filtered[IMU_AXIS_COUNT] */
void  ImuFilter_Update(const imu_sample_t *raw, float *filtered);

float ImuFilter_GetAlpha(imu_axis_t axis);
void  ImuFilter_SetAlpha(imu_axis_t axis, float alpha);

/* 按步长增减某个轴的系数，内部会限幅在 [MIN, MAX] 之间 */
void  ImuFilter_StepAlpha(imu_axis_t axis, int8_t direction);

#endif /* _IMU_FILTER_H_ */
