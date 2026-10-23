//
//  Quaternion attitude estimation - Mahony complementary filter
//  ----------------------------------------------------------------
//  File name kept as "madgwickFilter" to avoid touching the EIDE/Keil
//  project files; the algorithm inside has been REPLACED by Mahony.
//
//  Input : acceleration (g) + angular rate (rad/s), already axis-mapped
//          in imu_attitude.c
//  Output: q_est quaternion, converted to roll/pitch/yaw (deg) by eulerAngles()
//

#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <math.h>

// Sampling period (s). Must match the PIT interrupt rate: 100Hz -> 0.01s
#ifndef DELTA_T
#define DELTA_T 0.01f
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

// Mahony complementary-filter gains
//   KP : proportional gain (accelerometer correction strength)
//   KI : integral gain (gyro bias estimation). Keep 0 while the car is moving
//        because linear acceleration corrupts the gravity reference.
#define MAHONY_KP 2.0f
#define MAHONY_KI 0.0f

// Quaternion: q1 = scalar part (w), q2 = x, q3 = y, q4 = z
struct quaternion {
    float q1;
    float q2;
    float q3;
    float q4;
};

extern struct quaternion q_est;

void imu_filter(float ax, float ay, float az, float gx, float gy, float gz);
void eulerAngles(struct quaternion q, float *roll, float *pitch, float *yaw);

#endif /* MADGWICK_FILTER_H */
