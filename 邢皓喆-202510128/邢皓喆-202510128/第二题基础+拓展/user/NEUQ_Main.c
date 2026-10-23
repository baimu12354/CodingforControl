#include "NEUQ_lib_include.h"


void User_Main(void) {
    NEUQ_Delay_init();
    NEUQ_Debug_init();   
    Settings_Load();
    NEUQ_MANU_Init();
    MPU6050_Service();
    NEUQ_PIT_init_ms(PIT_CH1, KEY_SCAN_PERIOD);
    while (1) {
        MPU6050_Service();
        NEUQ_MANU_Task();
    }
}