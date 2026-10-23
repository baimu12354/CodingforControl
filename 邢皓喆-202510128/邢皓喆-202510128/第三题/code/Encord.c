#include "NEUQ_lib_include.h"
#define SPEED_FILTER_N  4U


static int32_t Count_GET(Encoder_DEV_ENUM DEV_select)
{
    int32_t count;
    count = NEUQ_Encoder_Read(DEV_select);
    NEUQ_Encoder_Clear(DEV_select);
    return count;
}
/*float Rpm_GET(Encoder_DEV_ENUM DEV_select)
{
    return -Count_GET(DEV_select) * 60.0f / (Counts_Sum * 0.01f);
}*/

/* 当前用于单个编码器，每 10 ms 调用一次 */
float Rpm_GET(Encoder_DEV_ENUM DEV_select)
{
    static int32_t history[SPEED_FILTER_N] = {0};
    static int32_t sum = 0;
    static uint8_t index = 0;
    static uint8_t samples = 0;

    int32_t count = Count_GET(DEV_select);

    sum -= history[index];
    history[index] = count;
    sum += count;

    index++;
    if (index >= SPEED_FILTER_N)
    {
        index = 0;
    }

    if (samples < SPEED_FILTER_N)
    {
        samples++;
    }

    return -(float)sum * 60.0f
           / (Counts_Sum * 0.01f * samples);
}
