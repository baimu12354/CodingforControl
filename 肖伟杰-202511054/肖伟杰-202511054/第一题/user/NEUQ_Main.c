#include "NEUQ_lib_include.h"
#include "NEUQ_Key.h"
#include "NEUQ_Menu.h"

/* 第一题：LED 时序控制（PA8），1 ms 节拍里跑 LED 时序，每 10 ms 扫按键 */
void User_Main(void)
{
    NEUQ_Menu_Init();               /* LED 引擎 + 按键 + LCD + 菜单首屏 */

    NEUQ_PIT_init_ms(PIT_CH1, 1);   /* 1 ms：LED 时序，分频 10 ms 扫按键 */

    while (1)
    {
        uint8_t i;

        for (i = 0U; i < (uint8_t)KEY_ID_NUM; i++)
        {
            KeyEventEnum event = NEUQ_Key_GetEvent((KeyIdEnum)i);
            if (event != KEY_EVENT_NONE)
            {
                NEUQ_Menu_HandleKey((KeyIdEnum)i, event);
            }
        }

        NEUQ_Menu_Task();
    }
}
