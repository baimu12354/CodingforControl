#ifndef __FLASH_H__
#define __FLASH_H__

#include "NEUQ_Main.h"

 extern volatile uint8_t flash_count ;
extern volatile uint8_t musave ;
void flash_load(void);


 void flash_save(void);
#endif 