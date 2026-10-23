#include "NEUQ_lib_include.h"

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float sample_period = 1.0f / sampleFreqDef;
static float beta = betaDef;
float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;
uint8_t imu_calibrating = 1;
float raw_accel[3] = {0.0f, 0.0f, 0.0f};
float raw_gyro[3]  = {0.0f, 0.0f, 0.0f};
float accel_data[3] = {0.0f, 0.0f, 0.0f};
float gyro_data[3]  = {0.0f, 0.0f, 0.0f};

float last_accel[3] = {0.0f, 0.0f, 0.0f};
float last_gyro[3]  = {0.0f, 0.0f, 0.0f};

int32_t filter_kp = 5;

float roll_angle = 0.0f;
float pitch_angle = 0.0f;
float yaw_angle = 0.0f;


static float invSqrt(float x)
{
    /* Zero vectors can occur during sensor calibration. */
    if (x <= 0.0f) return 0.0f; 
    return 1.0f / sqrtf(x);
}

void Madgwick_Init(void) {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    sample_period = 1.0f / sampleFreqDef;
    beta = betaDef;
}

static void computeAngles(void) {
    /* Clamp asin input against floating-point roundoff. */
    roll_angle  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f; 
    float sine_pitch = -2.0f * (q1*q3 - q0*q2);
    if (sine_pitch > 1.0f) sine_pitch = 1.0f;
    if (sine_pitch < -1.0f) sine_pitch = -1.0f;
    pitch_angle = asinf(sine_pitch) * 57.29578f;
    yaw_angle   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29578f;
}

void Madgwick_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    /* Gyroscope input is deg/s; acceleration input is g. */
    gx -=gyro_offset_x;
    gy -=gyro_offset_y;
    gz -=gyro_offset_z;


    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	/* Magnetometer input is currently zero; yaw has no absolute reference. */
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		/* Madgwick gradient correction. */
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	q0 += qDot1 * sample_period;
	q1 += qDot2 * sample_period;
	q2 += qDot3 * sample_period;
	q3 += qDot4 * sample_period;
    
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    computeAngles();
}
float LowPass_Filter(float new_val, float old_val, float alpha) {
    return alpha * new_val + (1.0f - alpha) * old_val;
}
uint8_t NEUQ_IMU_Task(void)
{
    static uint32_t sample_tick = 0;
    static uint16_t calib_count = 0;
    static float sum[3] = {0.0f, 0.0f, 0.0f};
    uint32_t now = HAL_GetTick();
    /* Unsigned subtraction handles HAL tick wraparound. */
    uint32_t elapsed = now - sample_tick;
    if (elapsed < 10) return 0;
    sample_tick = now;
    NEUQ_MPU6050_read_data();
    raw_accel[0] = MPU6050_acc_transition(mpu6050_acc_x);
    raw_accel[1] = MPU6050_acc_transition(mpu6050_acc_y);
    raw_accel[2] = MPU6050_acc_transition(mpu6050_acc_z);
    raw_gyro[0] = MPU6050_gyro_transition(mpu6050_gyro_x);
    raw_gyro[1] = MPU6050_gyro_transition(mpu6050_gyro_y);
    raw_gyro[2] = MPU6050_gyro_transition(mpu6050_gyro_z);
    if (imu_calibrating) {
        /* Gyroscope bias uses 100 stationary samples. */
        for (uint8_t i = 0; i < 3; i++) sum[i] += raw_gyro[i];
        if (++calib_count == 100) {
            gyro_offset_x = sum[0] / 100.0f;
            gyro_offset_y = sum[1] / 100.0f;
            gyro_offset_z = sum[2] / 100.0f;
            memcpy(last_accel, raw_accel, sizeof(last_accel));
            memcpy(last_gyro, raw_gyro, sizeof(last_gyro));
            imu_calibrating = 0;
        }
        return 0;
    }
    float alpha = filter_kp / 100.0f;
    for (uint8_t i = 0; i < 3; i++) {
        accel_data[i] = LowPass_Filter(raw_accel[i], last_accel[i], alpha);
        gyro_data[i] = LowPass_Filter(raw_gyro[i], last_gyro[i], alpha);
        last_accel[i] = accel_data[i];
        last_gyro[i] = gyro_data[i];
    }
    
    /* Use elapsed time to account for delays in the main loop. */
    sample_period = elapsed / 1000.0f;
    Madgwick_Update(gyro_data[0], gyro_data[1], gyro_data[2],
                   accel_data[0], accel_data[1], accel_data[2], 0.0f, 0.0f, 0.0f);
    return 1;
}
