#include "NEUQ_lib_include.h"


KeyConfig KEY = {
 KEY_Port, 0
};
static void KEY_CNT(void)
{
    TASK_CMD++;
    if (TASK_CMD > 4)
    {
       TASK_CMD = 1;
    }
}
void KEY_init(void)
{
    NEUQ_GPIO_init( KEY.Ports, GPIO_Mode_Input_PullUp,GPIO_High);
    NEUQ_PIT_init_ms(PIT_CH1, 1);
}

void KEY_Scan(void)
{   
    static GPIO_Level stable_level = GPIO_High;

    GPIO_Level current_level = NEUQ_GPIO_read(KEY.Ports);

    if (current_level == stable_level)
    {
        KEY.cnt = 0;      
        return;
    }
    KEY.cnt++;
    if (KEY.cnt >= KEY_CNT_MIN)
    {
        KEY.cnt = 0;
        stable_level = current_level;

        if (stable_level == GPIO_Low)
        {
            KEY_CNT();
        }
    }
}

