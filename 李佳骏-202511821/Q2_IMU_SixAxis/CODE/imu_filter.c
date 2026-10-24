/* imu_filter.c - 六路一阶低通 */

#include "imu_filter.h"
#include "board_config.h"

static float s_state[IMU_AXIS_COUNT];    /* 各轴上一次的滤波输出 */
static float s_alpha[IMU_AXIS_COUNT];    /* 各轴的滤波系数 */

static float ImuFilter_Clamp(float alpha)
{
    if (alpha < T2_FILTER_ALPHA_MIN)
    {
        return T2_FILTER_ALPHA_MIN;
    }
    if (alpha > T2_FILTER_ALPHA_MAX)
    {
        return T2_FILTER_ALPHA_MAX;
    }
    return alpha;
}

void ImuFilter_Init(void)
{
    uint8_t i;

    for (i = 0; i < IMU_AXIS_COUNT; i++)
    {
        s_alpha[i] = T2_FILTER_DEFAULT_ALPHA;
        s_state[i] = 0.0f;
    }
}

void ImuFilter_ResetState(void)
{
    uint8_t i;

    for (i = 0; i < IMU_AXIS_COUNT; i++)
    {
        s_state[i] = 0.0f;
    }
}

/* 六路独立一阶低通：y += α(x - y) */
void ImuFilter_Update(const imu_sample_t *raw, float *filtered)
{
    float input;
    uint8_t i;

    if (raw == 0 || filtered == 0)
    {
        return;
    }

    for (i = 0; i < IMU_AXIS_COUNT; i++)
    {
        input      = IMU_AxisValue(raw, (imu_axis_t)i);
        s_state[i] = s_state[i] + s_alpha[i] * (input - s_state[i]);
        filtered[i] = s_state[i];
    }
}

float ImuFilter_GetAlpha(imu_axis_t axis)
{
    if (axis >= IMU_AXIS_COUNT)
    {
        return T2_FILTER_DEFAULT_ALPHA;
    }
    return s_alpha[axis];
}

void ImuFilter_SetAlpha(imu_axis_t axis, float alpha)
{
    if (axis >= IMU_AXIS_COUNT)
    {
        return;
    }
    s_alpha[axis] = ImuFilter_Clamp(alpha);
}

void ImuFilter_StepAlpha(imu_axis_t axis, int8_t direction)
{
    float alpha;

    if (axis >= IMU_AXIS_COUNT)
    {
        return;
    }

    alpha = s_alpha[axis] + ((direction > 0) ? T2_FILTER_ALPHA_STEP : -T2_FILTER_ALPHA_STEP);
    s_alpha[axis] = ImuFilter_Clamp(alpha);
}
