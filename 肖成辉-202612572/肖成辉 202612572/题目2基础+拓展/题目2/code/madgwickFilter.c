//
//  Quaternion attitude estimation - Mahony complementary filter
//
//  Principle:
//    1. Integrate gyro angular rate to track attitude (fast, drifts over time)
//    2. Accelerometer measures gravity; cross-product with the gravity vector
//       predicted from the current quaternion gives the error
//    3. Feed that error back through a PI controller to correct the gyro
//
//  Input : ax/ay/az acceleration (g), gx/gy/gz angular rate (rad/s)
//  Output: q_est quaternion; eulerAngles() converts to roll/pitch/yaw (deg)
//

#include "madgwickFilter.h"

// Attitude quaternion, initialised to identity (no rotation)
struct quaternion q_est = { 1.0f, 0.0f, 0.0f, 0.0f };

// Gyro-bias integral terms (used only when MAHONY_KI > 0)
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;

void imu_filter(float ax, float ay, float az, float gx, float gy, float gz)
{
    float norm;
    float vx, vy, vz;   // gravity direction predicted from current quaternion
    float ex, ey, ez;   // error = measured acc cross predicted gravity

    // 1) Normalise accelerometer (direction only)
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 1e-6f) {
        ax /= norm; ay /= norm; az /= norm;
    }

    // 2) Predicted gravity direction in body frame
    vx = 2.0f * (q_est.q2 * q_est.q4 - q_est.q1 * q_est.q3);
    vy = 2.0f * (q_est.q1 * q_est.q2 + q_est.q3 * q_est.q4);
    vz = q_est.q1 * q_est.q1 - q_est.q2 * q_est.q2
       - q_est.q3 * q_est.q3 + q_est.q4 * q_est.q4;

    // 3) Error = measured acceleration cross predicted gravity
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 4) PI feedback to correct the gyro rate
    if (MAHONY_KI > 0.0f) {
        integralFBx += MAHONY_KI * ex * DELTA_T;
        integralFBy += MAHONY_KI * ey * DELTA_T;
        integralFBz += MAHONY_KI * ez * DELTA_T;
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    } else {
        integralFBx = integralFBy = integralFBz = 0.0f;
    }
    gx += MAHONY_KP * ex;
    gy += MAHONY_KP * ey;
    gz += MAHONY_KP * ez;

    // 5) First-order quaternion integration: q_dot = 0.5 * q (x) [0, gx, gy, gz]
    q_est.q1 += 0.5f * DELTA_T * (-q_est.q2 * gx - q_est.q3 * gy - q_est.q4 * gz);
    q_est.q2 += 0.5f * DELTA_T * ( q_est.q1 * gx + q_est.q3 * gz - q_est.q4 * gy);
    q_est.q3 += 0.5f * DELTA_T * ( q_est.q1 * gy - q_est.q2 * gz + q_est.q4 * gx);
    q_est.q4 += 0.5f * DELTA_T * ( q_est.q1 * gz + q_est.q2 * gy - q_est.q3 * gx);

    // 6) Normalise quaternion
    norm = sqrtf(q_est.q1 * q_est.q1 + q_est.q2 * q_est.q2
               + q_est.q3 * q_est.q3 + q_est.q4 * q_est.q4);
    if (norm > 1e-6f) {
        q_est.q1 /= norm; q_est.q2 /= norm;
        q_est.q3 /= norm; q_est.q4 /= norm;
    }
}

// Quaternion -> Euler angles (degrees). roll about X, pitch about Y, yaw about Z.
void eulerAngles(struct quaternion q, float *roll, float *pitch, float *yaw)
{
    *yaw   = atan2f(2.0f * q.q2 * q.q3 - 2.0f * q.q1 * q.q4,
                    2.0f * q.q1 * q.q1 + 2.0f * q.q2 * q.q2 - 1.0f);
    *pitch = -asinf(2.0f * q.q2 * q.q4 + 2.0f * q.q1 * q.q3);
    *roll  = atan2f(2.0f * q.q3 * q.q4 - 2.0f * q.q1 * q.q2,
                    2.0f * q.q1 * q.q1 + 2.0f * q.q4 * q.q4 - 1.0f);

    *yaw   *= 180.0f / PI;
    *pitch *= 180.0f / PI;
    *roll  *= 180.0f / PI;
}
