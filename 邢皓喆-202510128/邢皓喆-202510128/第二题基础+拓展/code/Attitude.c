#include "Attitude.h"
#include <math.h>
#include <string.h>

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define ATTITUDE_KP 2.0f
#define MAX_DT_SECONDS 0.05f

/* Coarse plausibility limits apply BEFORE bias is known. Stationarity is
 * checked separately over the whole window below. A measured static bias
 * near 7 deg/s must not prevent estimation of that same bias.
 * Keep the acceleration envelope consistent with the fusion feedback gate. */
#define CAL_ACC_MIN_G 0.8f
#define CAL_ACC_MAX_G 1.2f
#define CAL_MAX_RAW_GYRO_DPS 20.0f

static Attitude_Data attitude;
static volatile uint8_t calibration_request;
static uint16_t warmup_left;
static float mean[6];
static float gyro_m2[3];
static float acc_reference[3];

static void ClearCalibrationWindow(void)
{
    attitude.calibration_count = 0U;
    memset(mean, 0, sizeof(mean));
    memset(gyro_m2, 0, sizeof(gyro_m2));
}

void Attitude_Init(void)
{
    memset(&attitude, 0, sizeof(attitude));
    attitude.q[0] = 1.0f;
    attitude.status = ATTITUDE_CALIBRATING;
    warmup_left = ATTITUDE_WARMUP_SAMPLES;
    calibration_request = 0U;
    ClearCalibrationWindow();
}

void Attitude_RequestCalibration(void)
{
    calibration_request = 1U;
}

void Attitude_OnSensorError(void)
{
    attitude.status = ATTITUDE_SENSOR_ERROR;
    attitude.dt_ms = 0.0f;
    ClearCalibrationWindow();
}

void Attitude_GetData(Attitude_Data *out)
{
    if (out != 0) {
        *out = attitude;
    }
}

static void UpdateEuler(void)
{
    float w = attitude.q[0], x = attitude.q[1];
    float y = attitude.q[2], z = attitude.q[3];
    float s = 2.0f * (w*y - z*x);
    /* 浮点舍入可能使 s 略超出 [-1,1]，必须限幅后调用 asinf。 */
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    attitude.roll = atan2f(2.0f*(w*x + y*z),
                          1.0f - 2.0f*(x*x + y*y)) * RAD_TO_DEG;
    attitude.pitch = asinf(s) * RAD_TO_DEG;
    attitude.yaw = atan2f(2.0f*(w*z + x*y),
                         1.0f - 2.0f*(y*y + z*z)) * RAD_TO_DEG;
}

static void Calibrate(const float data[6], float acc_norm)
{
    unsigned int i;
    float delta, r, p, cr, sr, cp, sp;
    /* 先丢弃启动瞬态，再采集连续的静止样本。没有阻塞延时。 */
    if (warmup_left != 0U) {
        --warmup_left;
        return;
    }
    if (acc_norm <= CAL_ACC_MIN_G || acc_norm >= CAL_ACC_MAX_G ||
        fabsf(data[3]) > CAL_MAX_RAW_GYRO_DPS ||
        fabsf(data[4]) > CAL_MAX_RAW_GYRO_DPS ||
        fabsf(data[5]) > CAL_MAX_RAW_GYRO_DPS) {
        ClearCalibrationWindow();
        return;
    }
    if (attitude.calibration_count == 0U) {
        for (i = 0; i < 3U; ++i) acc_reference[i] = data[i];
    }
    /* 相对本窗口起点检查方向变化，避免缓慢倾斜被当成静止。 */
    for (i = 0; i < 3U; ++i) {
        if (fabsf(data[i] - acc_reference[i]) > 0.04f) {
            ClearCalibrationWindow();
            return;
        }
    }
    ++attitude.calibration_count;
    for (i = 0; i < 6U; ++i) {
        delta = data[i] - mean[i];
        mean[i] += delta / (float)attitude.calibration_count;
        if (i >= 3U) gyro_m2[i-3U] += delta * (data[i] - mean[i]);
    }
    if (attitude.calibration_count < ATTITUDE_CALIBRATION_SAMPLES) return;

    /* 标准差超过 0.15 deg/s 则重新采集，门限需结合实物噪声调整。
     * 恒定的慢速旋转仍可能被误认为偏置，所以校准时必须放稳模块。 */
    for (i = 0; i < 3U; ++i) {
        if (gyro_m2[i] / (float)(ATTITUDE_CALIBRATION_SAMPLES-1U) > 0.0225f) {
            ClearCalibrationWindow();
            return;
        }
    }
    for (i = 0; i < 3U; ++i) attitude.gyro_bias[i] = mean[i+3U];

    /* 用静止加速度初始化实际倾斜；航向定为 0。
     * 加速度中的 1g 是方向参考，不能像陀螺仪一样全部减掉。 */
    r = atan2f(mean[1], mean[2]);
    p = atan2f(-mean[0], sqrtf(mean[1]*mean[1] + mean[2]*mean[2]));
    cr = cosf(r*0.5f); sr = sinf(r*0.5f);
    cp = cosf(p*0.5f); sp = sinf(p*0.5f);
    attitude.q[0] = cr*cp;
    attitude.q[1] = sr*cp;
    attitude.q[2] = cr*sp;
    attitude.q[3] = -sr*sp;
    UpdateEuler();
    attitude.status = ATTITUDE_READY;
}

void Attitude_ProcessSample(const float data[6], float dt_seconds)
{
    unsigned int i;
    float ax, ay, az, gx, gy, gz, n;
    float w, x, y, z, vx, vy, vz, ex, ey, ez, half_dt;
    if (calibration_request || attitude.status == ATTITUDE_SENSOR_ERROR) {
        Attitude_Init();
    }
    if (data == 0) {
        Attitude_OnSensorError();
        return;
    }
    /* 比较写法同时拒绝 NaN/Inf，避免一次坏数据污染后续所有姿态。 */
    for (i = 0; i < 6U; ++i) {
        float limit = (i < 3U) ? 16.0f : 2100.0f;
        if (!(data[i] >= -limit && data[i] <= limit)) {
            Attitude_OnSensorError();
            return;
        }
    }
    if (!(dt_seconds > 0.0f && dt_seconds <= MAX_DT_SECONDS)) {
        /* 丢失较长一段运动不能靠恢复后的单个样本补积分。 */
        Attitude_Init();
        return;
    }
    attitude.dt_ms = dt_seconds * 1000.0f;
    ax = data[0]; ay = data[1]; az = data[2];
    n = sqrtf(ax*ax + ay*ay + az*az);
    if (attitude.status != ATTITUDE_READY) {
        Calibrate(data, n);
        return;
    }

    /* 1. 三轴零偏补偿，然后 deg/s -> rad/s。 */
    gx = (data[3] - attitude.gyro_bias[0]) * DEG_TO_RAD;
    gy = (data[4] - attitude.gyro_bias[1]) * DEG_TO_RAD;
    gz = (data[5] - attitude.gyro_bias[2]) * DEG_TO_RAD;
    w = attitude.q[0]; x = attitude.q[1];
    y = attitude.q[2]; z = attitude.q[3];

    /* 2. Mahony 比例反馈：实测方向叉乘预测方向，纠正倾斜漂移。
     * 模长明显偏离 1g 时停用加速度反馈，但继续陀螺仪积分。
     * 此门限不能排除所有线性加速度，适合题目中的缓慢倾斜测试。 */
    if (n > 0.8f && n < 1.2f) {
        ax /= n; ay /= n; az /= n;
        vx = 2.0f*(x*z - w*y);
        vy = 2.0f*(w*x + y*z);
        vz = w*w - x*x - y*y + z*z;
        ex = ay*vz - az*vy;
        ey = az*vx - ax*vz;
        ez = ax*vy - ay*vx;
        gx += ATTITUDE_KP*ex;
        gy += ATTITUDE_KP*ey;
        gz += ATTITUDE_KP*ez;
    }

    /* 3. q_dot = 0.5 * q ⊗ [0, omega]，全部使用同一组旧四元数。 */
    half_dt = 0.5f * dt_seconds;
    attitude.q[0] = w + (-x*gx - y*gy - z*gz)*half_dt;
    attitude.q[1] = x + ( w*gx + y*gz - z*gy)*half_dt;
    attitude.q[2] = y + ( w*gy - x*gz + z*gx)*half_dt;
    attitude.q[3] = z + ( w*gz + x*gy - y*gx)*half_dt;

    /* 4. 归一化，然后仅为显示转换成欧拉角。 */
    n = sqrtf(attitude.q[0]*attitude.q[0] + attitude.q[1]*attitude.q[1] +
              attitude.q[2]*attitude.q[2] + attitude.q[3]*attitude.q[3]);
    if (!(n > 0.000001f && n < 100.0f)) {
        Attitude_Init();
        return;
    }
    for (i = 0; i < 4U; ++i) attitude.q[i] /= n;
    UpdateEuler();
}
