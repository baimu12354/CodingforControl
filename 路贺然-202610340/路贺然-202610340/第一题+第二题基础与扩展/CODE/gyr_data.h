#ifndef NEUQ_GYR_DATA_H_
#define NEUQ_GYR_DATA_H_

#include "NEUQ_lib_include.h"
extern int16_t ax_raw, ay_raw, az_raw;
extern int16_t gx_raw, gy_raw, gz_raw;
extern float acc_x_g, acc_y_g, acc_z_g;
extern float gyro_x_dps, gyro_y_dps, gyro_z_dps;
void MPU6050_Measure_6Axis(void);

extern volatile uint8_t g_wave_tick_req;   // 编辑完成
#endif