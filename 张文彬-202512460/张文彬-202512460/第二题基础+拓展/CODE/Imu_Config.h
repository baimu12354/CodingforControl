#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/* 原工程引脚、周期、Flash 格式与共享数据类型，数值保持不变。 */
#include <stdint.h>

#define APP_KEY_UP                    PE9
#define APP_KEY_DOWN                  PE8
#define APP_KEY_OK                    PE7
#define APP_KEY_BACK                  PE10
#define APP_KEY_COUNT                 4U

#define APP_SAMPLE_MS                 10UL
#define APP_LCD_MS                    100UL
#define APP_KEY_DEBOUNCE_MS           20UL
#define APP_FLASH_SECTOR              NEUQ_FLASH_LAST_SECTOR_ID
#define APP_CONFIG_MAGIC              0x32464C50UL  /* "2FLP" */

typedef enum
{
    KEY_UP = 0,
    KEY_DOWN,
    KEY_OK,
    KEY_BACK
} AppKey;

typedef enum
{
    PAGE_HOME = 0,
    PAGE_SIX_AXIS,
    PAGE_FILTER,
    PAGE_ATTITUDE
} AppPage;

typedef struct
{
    uint32_t magic;
    uint16_t alpha_milli;     /* 50~950 对应 0.05~0.95 */
    uint8_t send_axis;        /* 0~5：AX AY AZ GX GY GZ */
    uint8_t reserved;
    uint32_t checksum;
} AppConfig;

typedef struct
{
    float raw[6];
    float filtered[6];
    uint8_t initialized;
} ImuData;

#endif /* IMU_CONFIG_H */
