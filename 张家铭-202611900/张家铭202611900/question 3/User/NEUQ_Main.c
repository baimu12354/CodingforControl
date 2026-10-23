
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Motor.h"

void User_Main(void)
{
    NEUQ_Debug_init(); 
    NEUQ_Delay_init(); 
    Motor_Init();

    while (1) {
        Motor_Task();
    }
}
