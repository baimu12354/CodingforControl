/* imu_attitude.c - 四元数姿态解算 */

#include "imu_attitude.h"
#include "board_config.h"
#include <math.h>

#define ATT_DEG2RAD     (0.01745329252f)
#define ATT_RAD2DEG     (57.29577951f)

static attitude_t s_att;
static uint8_t    s_first_call = 1;
static uint32_t   s_last_ms;

/* 校准过程的临时变量 */
static uint32_t   s_cal_ms;
static uint32_t   s_cal_count;
static float      s_cal_sum[3];
static float      s_cal_acc[3];

/* 加速度计的固定低通状态（归一化之后的值） */
static float      s_acc_lpf[3];

static void Attitude_MapAxes(const float *in, float *out)
{
    out[0] = T2_ATT_ROLL_SIGN  * in[T2_ATT_ROLL_AXIS];
    out[1] = T2_ATT_PITCH_SIGN * in[T2_ATT_PITCH_AXIS];
    out[2] = T2_ATT_YAW_SIGN   * in[T2_ATT_YAW_AXIS];
}

static void Attitude_ResetSolver(void)
{
    uint8_t i;

    s_att.q[0] = 1.0f;
    s_att.q[1] = 0.0f;
    s_att.q[2] = 0.0f;
    s_att.q[3] = 0.0f;
    s_att.roll  = 0.0f;
    s_att.pitch = 0.0f;
    s_att.yaw   = 0.0f;
    s_att.updates = 0;

    for (i = 0; i < 3U; i++)
    {
        s_acc_lpf[i] = 0.0f;
    }
}

void Attitude_Init(void)
{
    uint8_t i;

    s_att.roll     = 0.0f;
    s_att.pitch    = 0.0f;
    s_att.yaw      = 0.0f;
    s_att.period_ms = 0;
    s_att.updates  = 0;
    for (i = 0; i < 3U; i++)
    {
        s_att.gyro_bias[i] = 0.0f;
    }
    Attitude_ResetSolver();

    s_first_call = 1;
    Attitude_StartCalibration();
}

void Attitude_StartCalibration(void)
{
    s_att.state    = ATT_STATE_WAIT;
    s_att.progress = 0;
    s_cal_ms       = 0;
    s_cal_count    = 0;
    s_cal_sum[0]   = 0.0f;
    s_cal_sum[1]   = 0.0f;
    s_cal_sum[2]   = 0.0f;
    s_cal_acc[0]   = 0.0f;
    s_cal_acc[1]   = 0.0f;
    s_cal_acc[2]   = 0.0f;
    s_first_call   = 1;         /* 下一次调用重新起算时间 */
}

/* 用校准后的平均加速度初始化 roll/pitch */
static void Attitude_InitFromAccel(void)
{
    float avg[3];
    float body[3];
    float norm;
    float roll0, pitch0;
    float cp, sp, cr, sr;
    uint8_t i;

    for (i = 0; i < 3U; i++)
    {
        avg[i] = s_cal_acc[i] / (float)s_cal_count;
    }
    Attitude_MapAxes(avg, body);

    norm = sqrtf(body[0] * body[0] + body[1] * body[1] + body[2] * body[2]);
    if (norm < 0.001f)
    {
        return;                     /* 数据异常，保持单位四元数 */
    }
    body[0] /= norm;
    body[1] /= norm;
    body[2] /= norm;

    /* 机体 Z 轴朝上时，加速度计读数是 (0, 0, 1)。
     * 绕横滚轴转 r 后是 (0, sin r, cos r)；再绕俯仰轴转 p 后是 (-sin p, ..., ...)。
     * 所以：横滚 = atan2(ay, az)，俯仰 = asin(-ax)。 */
    roll0 = atan2f(body[1], body[2]);
    pitch0 = -body[0];
    if (pitch0 >  1.0f) { pitch0 =  1.0f; }
    if (pitch0 < -1.0f) { pitch0 = -1.0f; }
    pitch0 = asinf(pitch0);

    /* 按 ZYX 顺序构造四元数：q = qy(pitch) ⊗ qx(roll)，航向取 0 */
    cp = cosf(pitch0 * 0.5f);
    sp = sinf(pitch0 * 0.5f);
    cr = cosf(roll0 * 0.5f);
    sr = sinf(roll0 * 0.5f);

    s_att.q[0] =  cp * cr;
    s_att.q[1] =  cp * sr;
    s_att.q[2] =  cr * sp;
    s_att.q[3] = -sp * sr;

    s_att.roll  = roll0  * ATT_RAD2DEG;
    s_att.pitch = pitch0 * ATT_RAD2DEG;
    s_att.yaw   = 0.0f;

    for (i = 0; i < 3U; i++)
    {
        s_acc_lpf[i] = body[i];     /* 低通初值直接取当前方向，省掉一段过渡 */
    }
}

/* 零偏校准：2 s 准备 + 2 s 采样，晃动则重来 */
static void Attitude_CalibrationStep(const imu_sample_t *sample, uint32_t now_ms)
{
    float    gx, gy, gz, mag;
    uint32_t dt_ms;
    uint8_t  i;

    if (s_first_call != 0U)
    {
        s_first_call = 0;
        s_last_ms    = now_ms;
        return;
    }

    dt_ms     = now_ms - s_last_ms;
    s_last_ms = now_ms;
    if (dt_ms > T2_ATT_MAX_DT_MS)
    {
        dt_ms = T2_ATT_MAX_DT_MS;
    }
    s_cal_ms += dt_ms;

    /* 阶段一：等板子放平 */
    if (s_att.state == ATT_STATE_WAIT)
    {
        if (s_cal_ms >= T2_ATT_CAL_PREPARE_MS)
        {
            s_att.state = ATT_STATE_SAMPLING;
            s_cal_ms    = 0;
            s_cal_count = 0;
            s_cal_sum[0] = 0.0f;
            s_cal_sum[1] = 0.0f;
            s_cal_sum[2] = 0.0f;
            s_cal_acc[0] = 0.0f;
            s_cal_acc[1] = 0.0f;
            s_cal_acc[2] = 0.0f;
        }
        s_att.progress = 0;
        return;
    }

    /* 阶段二：采零偏 */
    gx  = sample->gyro[0];
    gy  = sample->gyro[1];
    gz  = sample->gyro[2];
    mag = sqrtf(gx * gx + gy * gy + gz * gz);

    if (mag > T2_ATT_CAL_MOTION_DPS)
    {
        /* 检测到晃动：清零重来，并在屏幕上提示保持静止 */
        s_cal_ms    = 0;
        s_cal_count = 0;
        s_cal_sum[0] = 0.0f;
        s_cal_sum[1] = 0.0f;
        s_cal_sum[2] = 0.0f;
        s_cal_acc[0] = 0.0f;
        s_cal_acc[1] = 0.0f;
        s_cal_acc[2] = 0.0f;
        s_att.progress = 0;
        return;
    }

    s_cal_sum[0] += gx;
    s_cal_sum[1] += gy;
    s_cal_sum[2] += gz;
    s_cal_acc[0] += sample->acc[0];
    s_cal_acc[1] += sample->acc[1];
    s_cal_acc[2] += sample->acc[2];
    s_cal_count++;

    s_att.progress = (uint8_t)((s_cal_ms * 100U) / T2_ATT_CAL_SAMPLE_MS);
    if (s_att.progress > 100U)
    {
        s_att.progress = 100U;
    }

    if ((s_cal_ms >= T2_ATT_CAL_SAMPLE_MS) && (s_cal_count > 10U))
    {
        for (i = 0; i < 3U; i++)
        {
            s_att.gyro_bias[i] = s_cal_sum[i] / (float)s_cal_count;
        }
        Attitude_ResetSolver();
        Attitude_InitFromAccel();    /* 航向以此刻为 0，横滚俯仰直接对准 */
        s_att.state    = ATT_STATE_READY;
        s_att.progress = 100;
    }
}

/* 陀螺仪积分 + 加速度计重力修正 */
static void Attitude_Solve(const imu_sample_t *sample, uint32_t now_ms)
{
    float    gyro[3];
    float    acc[3];
    float    g_raw[3];
    float    a_raw[3];
    float    norm;
    float    dt;
    float    q0, q1, q2, q3;
    float    vx, vy, vz;
    float    ex, ey, ez;
    float    dq0, dq1, dq2, dq3;
    float    sinp;
    uint32_t dt_ms;
    uint8_t  i;

    if (s_first_call != 0U)
    {
        s_first_call = 0;
        s_last_ms    = now_ms;
        return;
    }

    dt_ms = now_ms - s_last_ms;
    if (dt_ms == 0U)
    {
        return;                     /* 同一毫秒内重复调用，积分没有意义 */
    }
    if (dt_ms > T2_ATT_MAX_DT_MS)
    {
        dt_ms = T2_ATT_MAX_DT_MS;   /* 主循环被刷屏阻塞时不至于一次积分太猛 */
    }
    s_last_ms      = now_ms;
    s_att.period_ms = dt_ms;
    dt             = (float)dt_ms * 0.001f;

    /* 1. 陀螺仪：减零偏、换轴、转弧度 */
    for (i = 0; i < 3U; i++)
    {
        g_raw[i] = sample->gyro[i] - s_att.gyro_bias[i];
    }
    Attitude_MapAxes(g_raw, gyro);
    gyro[0] *= ATT_DEG2RAD;
    gyro[1] *= ATT_DEG2RAD;
    gyro[2] *= ATT_DEG2RAD;

    /* 2. 加速度计：换轴、归一化、轻度低通 */
    Attitude_MapAxes(sample->acc, a_raw);
    norm = sqrtf(a_raw[0] * a_raw[0] + a_raw[1] * a_raw[1] + a_raw[2] * a_raw[2]);
    if (norm > 0.001f)
    {
        a_raw[0] /= norm;
        a_raw[1] /= norm;
        a_raw[2] /= norm;

        for (i = 0; i < 3U; i++)
        {
            /* 固定系数的一阶低通，专门用来压加速度计的噪声。
             * 系数写死在这里，不受菜单里滤波设置的影响。 */
            s_acc_lpf[i] = s_acc_lpf[i] + T2_ATT_ACC_LPF_ALPHA * (a_raw[i] - s_acc_lpf[i]);
        }
        norm = sqrtf(s_acc_lpf[0] * s_acc_lpf[0] +
                     s_acc_lpf[1] * s_acc_lpf[1] +
                     s_acc_lpf[2] * s_acc_lpf[2]);
        if (norm > 0.001f)
        {
            acc[0] = s_acc_lpf[0] / norm;
            acc[1] = s_acc_lpf[1] / norm;
            acc[2] = s_acc_lpf[2] / norm;
        }
        else
        {
            acc[0] = 0.0f;
            acc[1] = 0.0f;
            acc[2] = 1.0f;
        }

        /* 3. 用四元数预测重力方向，与加速度计方向做叉积得误差 */
        q0 = s_att.q[0]; q1 = s_att.q[1]; q2 = s_att.q[2]; q3 = s_att.q[3];
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        ex = acc[1] * vz - acc[2] * vy;
        ey = acc[2] * vx - acc[0] * vz;
        ez = acc[0] * vy - acc[1] * vx;

        /* 4. 把误差按比例加回角速度 */
        gyro[0] += T2_ATT_ACC_TRUST * ex;
        gyro[1] += T2_ATT_ACC_TRUST * ey;
        gyro[2] += T2_ATT_ACC_TRUST * ez;
    }

    /* 5. 四元数一阶积分 */
    q0 = s_att.q[0]; q1 = s_att.q[1]; q2 = s_att.q[2]; q3 = s_att.q[3];

    dq0 = 0.5f * (-q1 * gyro[0] - q2 * gyro[1] - q3 * gyro[2]) * dt;
    dq1 = 0.5f * ( q0 * gyro[0] + q2 * gyro[2] - q3 * gyro[1]) * dt;
    dq2 = 0.5f * ( q0 * gyro[1] - q1 * gyro[2] + q3 * gyro[0]) * dt;
    dq3 = 0.5f * ( q0 * gyro[2] + q1 * gyro[1] - q2 * gyro[0]) * dt;

    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;

    /* 6. 归一化（题目明确要求） */
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0001f)
    {
        s_att.q[0] = q0 / norm;
        s_att.q[1] = q1 / norm;
        s_att.q[2] = q2 / norm;
        s_att.q[3] = q3 / norm;
    }

    /* 7. 四元数转欧拉角 */
    q0 = s_att.q[0]; q1 = s_att.q[1]; q2 = s_att.q[2]; q3 = s_att.q[3];

    sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp >  1.0f) { sinp =  1.0f; }
    if (sinp < -1.0f) { sinp = -1.0f; }

    s_att.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2)) * ATT_RAD2DEG;
    s_att.pitch = asinf(sinp) * ATT_RAD2DEG;
    s_att.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                         1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ATT_RAD2DEG;

    s_att.updates++;
}

/* 校准阶段走状态机，完成后走解算 */
void Attitude_Update(const imu_sample_t *sample, uint32_t now_ms)
{
    if (sample == 0)
    {
        return;
    }

    if (s_att.state != ATT_STATE_READY)
    {
        Attitude_CalibrationStep(sample, now_ms);
        return;
    }

    Attitude_Solve(sample, now_ms);
}

const attitude_t *Attitude_Get(void)
{
    return &s_att;
}
