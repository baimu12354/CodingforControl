#include "NEUQ_lib_include.h"

#define SETTINGS_MAGIC 0x4C504631u
/* Board library reserves W25Q16 sector 511 (0x1FF000) for settings. */
#define SETTINGS_SECTOR_ID NEUQ_FLASH_LAST_SECTOR_ID

static uint32_t Settings_Address(void)
{
    static uint8_t initialized = 0;

    if (!initialized &&
        NEUQ_Flash_init() != NEUQ_FLASH_OK)
    {
        return NEUQ_FLASH_INVALID_ADDRESS;
    }
    initialized = 1;
    return NEUQ_Flash_get_sector_address(SETTINGS_SECTOR_ID);
}

uint8_t Setting_Save(void)
{
    uint32_t address;
    uint32_t settings[3];

    if (!(mpu_filter_alpha >= 0.01f && mpu_filter_alpha <= 1.0f))
    {
        return 0;
    }
    address = Settings_Address();
    if (address == NEUQ_FLASH_INVALID_ADDRESS)
    {
        return 0;
    }

    settings[0] = SETTINGS_MAGIC;
    settings[1] = (uint32_t)(mpu_filter_alpha * 100.0f + 0.5f);
    settings[2] = ~settings[1];
    /* The library preserves other bytes and verifies the saved data. */
    return NEUQ_Flash_save(address, (const uint8_t *)settings, sizeof(settings)) == NEUQ_FLASH_OK;
}

void Settings_Load(void)
{
    uint32_t address;
    uint32_t settings[3];

    mpu_filter_alpha = 0.2f;
    address = Settings_Address();
    if (address == NEUQ_FLASH_INVALID_ADDRESS ||
        NEUQ_Flash_read(address, (uint8_t *)settings,
                        sizeof(settings)) != NEUQ_FLASH_OK)
    {
        return;
    }

    if (settings[0] == SETTINGS_MAGIC &&
        settings[1] >= 1U && settings[1] <= 100U &&
        settings[2] == ~settings[1])
    {
        mpu_filter_alpha = settings[1] / 100.0f;
    }
}
