#include "param.h"

Param_t param;
uint32_t last_save_tick;                     /* 上次保存时刻（ms） */

void Param_Init(void) {
    NEUQ_Flash_init();
}

/* ================= Flash 参数读写 ================= */
void Param_Load(void) {
    Param_t t;
    param = (Param_t){0, 20, 0, 0, 0};
    if (NEUQ_Flash_read_sector(PARAM_SECTOR, (uint8_t *)&t, sizeof(t)) == NEUQ_FLASH_OK && t.magic == PARAM_MAGIC) {
        if (t.filter_percent >= 1 && t.filter_percent <= 100) param.filter_percent = t.filter_percent;
        if (t.send_axis < AX_NUM) param.send_axis = t.send_axis;
        param.send_enable = t.send_enable ? 1 : 0;
    }
}
void Param_Save(void) {
    param.magic = PARAM_MAGIC;
    NEUQ_Flash_write_sector(PARAM_SECTOR, (uint8_t *)&param, sizeof(param));
    last_save_tick = time_tick;
}
