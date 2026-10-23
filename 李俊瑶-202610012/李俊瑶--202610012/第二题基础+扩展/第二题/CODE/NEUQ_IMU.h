#ifndef _NEUQ_IMU_H_
#define _NEUQ_IMU_H_

#include <stdint.h>
#include <math.h>

#define sampleFreqDef 100.0f
#define betaDef 0.1f

extern float roll_angle;
extern float pitch_angle;
extern float yaw_angle;
extern uint8_t imu_calibrating;
void Madgwick_Init(void);
uint8_t NEUQ_IMU_Task(void);
/* gx/gy/gz: deg/s; ax/ay/az */
void Madgwick_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

extern float raw_accel[3];
extern float raw_gyro[3];
extern float accel_data[3];
extern float gyro_data[3];
extern float last_accel[3];
extern float last_gyro[3];
extern int32_t filter_kp;
float LowPass_Filter (float new_val, float old_val, float alpha);
#endif 
