#include "NEUQ_lib_include.h"

/**
 * @brief  用户主程序入口
 */
void User_Main(void) {
    NEUQ_Delay_init();
     NEUQ_Debug_init();                              
     LED_init();               
     LED_RUN_init();       
     KEY_init();
     NEUQ_LCD200_init();
    while (1) {
        UI_init();
    }
}