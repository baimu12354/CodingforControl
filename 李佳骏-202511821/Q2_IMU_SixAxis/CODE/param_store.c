/* param_store.c - 滤波参数 Flash 存储 */

#include "param_store.h"
#include "board_config.h"
#include "imu_filter.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

static uint32_t ParamStore_Checksum(const imu_params_t *p)
{
    const uint8_t *bytes = (const uint8_t *)p;
    uint32_t       sum   = 0;
    uint16_t       i;
    uint16_t       len   = (uint16_t)(sizeof(imu_params_t) - sizeof(uint32_t));

    for (i = 0; i < len; i++)
    {
        sum += bytes[i];
    }
    return sum;
}

/* 读 Flash，magic/checksum 不对则用默认值 */
uint8_t ParamStore_LoadAndApply(void)
{
    imu_params_t param;
    uint16_t     i;

    for (i = 0; i < sizeof(param); i++)
    {
        ((uint8_t *)&param)[i] = 0;
    }

    if (NEUQ_Flash_init() != NEUQ_FLASH_OK)
    {
        return 0;
    }

    if (NEUQ_Flash_read_sector((uint16_t)T2_PARAM_SECTOR,
                               (uint8_t *)&param,
                               (uint16_t)sizeof(param)) != NEUQ_FLASH_OK)
    {
        return 0;
    }

    if (param.magic != T2_PARAM_MAGIC)
    {
        return 0;                       /* 空白扇区或别的程序写过的数据 */
    }
    if (param.checksum != ParamStore_Checksum(&param))
    {
        return 0;                       /* 数据损坏 */
    }

    for (i = 0; i < IMU_AXIS_COUNT; i++)
    {
        ImuFilter_SetAlpha((imu_axis_t)i, param.alpha[i]);
    }
    return 1;
}

/* 六个轴系数写入 511 扇区 */
uint8_t ParamStore_SaveCurrent(void)
{
    imu_params_t param;
    uint8_t      i;

    param.magic = T2_PARAM_MAGIC;
    for (i = 0; i < IMU_AXIS_COUNT; i++)
    {
        param.alpha[i] = ImuFilter_GetAlpha((imu_axis_t)i);
    }
    param.checksum = ParamStore_Checksum(&param);

    return NEUQ_Flash_write_sector((uint16_t)T2_PARAM_SECTOR,
                                   (const uint8_t *)&param,
                                   (uint16_t)sizeof(param));
}
