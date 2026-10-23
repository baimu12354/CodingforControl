 #include "NEUQ_Main.h"
 #include <math.h>
 #include "menu.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
volatile uint8_t flash_count = 0;
volatile uint8_t musave = 0;
 
void flash_load(void)
{

    if(  NEUQ_Flash_read_sector(NEUQ_FLASH_LAST_SECTOR_ID, (uint8_t *)&saved, sizeof(saved)) == NEUQ_FLASH_OK  
         && saved.filter_alfa_acc >= TASK2_FILTER_MIN && saved.filter_alfa_acc <= TASK2_FILTER_MAX
         && saved.filter_alfa_gyro >= TASK2_FILTER_MIN && saved.filter_alfa_gyro <= TASK2_FILTER_MAX
         && saved.axis <= AXIS_NONE
      ) 
    { 
        filter_alfa_acc = saved.filter_alfa_acc;
        filter_alfa_gyro = saved.filter_alfa_gyro;
        vofa_axis = (AXIS)saved.axis;
        if(vofa_axis != AXIS_NONE)
        {
       
            menu_value[6+vofa_axis]=saved.value;
        }
    }
 
}



 void flash_save(void)
{

    saved.filter_alfa_acc = filter_alfa_acc;
    saved.filter_alfa_gyro = filter_alfa_gyro;
    saved.axis = (uint8_t)vofa_axis;
    saved.value = 0;
     if (vofa_axis != AXIS_NONE) 
     {
        saved.value = (*menu_table[MENU_AX + (uint8_t)vofa_axis].value);
     }
   NEUQ_Flash_write_sector(NEUQ_FLASH_LAST_SECTOR_ID, ( uint8_t *)&saved, sizeof(saved));
}