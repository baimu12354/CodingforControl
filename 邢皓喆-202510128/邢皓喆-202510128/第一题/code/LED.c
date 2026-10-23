#include "NEUQ_lib_include.h"


void LED_init(void)
{
    NEUQ_GPIO_init(LED_Pin, GPIO_Mode_Output_PushPull, GPIO_Low);
}
