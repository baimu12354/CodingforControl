#ifndef __FLASH_PARAM_H
#define __FLASH_PARAM_H

#include "NEUQ_lib_include.h"

#define PARAM_SECTOR (NEUQ_FLASH_LAST_SECTOR_ID)

void flash_param_load(void);
uint8_t flash_param_save(void);




#endif