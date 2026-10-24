/*********************************************************************************************************************
* 文件名称   config
* 功能说明   参数全局变量定义 + Flash 存取实现
*********************************************************************************************************************/
#include "config.h"

#define FLASH_CFG_SECTOR 510u          // 专用扇区，避开演示占用的511
#define FLASH_CFG_MAGIC  0x4E455551u   // 任意固定值，判断记录是否有效

FilterConfig_t g_cfg = {0};
int16_t g_acc[3]  = {0};
int16_t g_gyro[3] = {0};
float   g_acc_g[3]  = {0};
float   g_gyro_rad[3] = {0};
float   g_gyro_bias[3] = {0};
float   g_roll = 0.0f;
float   g_pitch = 0.0f;
float   g_yaw = 0.0f;
float   g_acc_f[3]  = {0};
float   g_gyro_f[3] = {0};
volatile uint8_t g_save_flag = 0;
volatile uint8_t g_upload_tick = 0;

void Config_Load(void)
{
    FilterConfig_t tmp;

    if (NEUQ_Flash_read_sector(FLASH_CFG_SECTOR, (uint8_t *)&tmp, (uint16_t)sizeof(tmp)) != NEUQ_FLASH_OK
        || tmp.magic != FLASH_CFG_MAGIC) {
        // Flash为空或记录损坏 -> 使用合理默认值
        g_cfg.magic         = FLASH_CFG_MAGIC;
        g_cfg.send_enable   = 0;
        g_cfg.selected_axis = 0;
        g_cfg.alpha_x1000   = 100;   // 默认低通系数 0.100
    } else {
        g_cfg = tmp;                  // 有效记录 -> 恢复
    }
}

uint8_t Config_Save(void)
{
    g_cfg.magic = FLASH_CFG_MAGIC;
    return NEUQ_Flash_write_sector(FLASH_CFG_SECTOR, (uint8_t *)&g_cfg, (uint16_t)sizeof(g_cfg));
}
