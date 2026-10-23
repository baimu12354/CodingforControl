#include "NEUQ_lib_include.h"
#include "NEUQ_Key.h"
#include "NEUQ_Menu2.h"
#include "NEUQ_IMU.h"
#include "NEUQ_Attitude.h"

/* 第二题：六轴数据处理 + 四元数姿态解算 */
void User_Main(void)
{
    uint8_t imu_ok;

    NEUQ_Debug_init();
    NEUQ_Delay_init();

    NEUQ_Menu2_Init();              /* LCD + 菜单 + Flash 参数 + VOFA 串口 */
    imu_ok = NEUQ_IMU_Init();       /* MPU6050 + 滤波系数 */
    NEUQ_Attitude_Init();           /* 四元数按当前重力方向归零 */

    NEUQ_Key_Init();
    NEUQ_PIT_init_ms(PIT_CH1, 1);   /* 1 ms：10 ms 扫按键 */
    NEUQ_PIT_init_ms(PIT_CH2, 5);   /* 5 ms：六轴采集 + 滤波 + 姿态 */

    if (imu_ok != 0U)
    {
        uint32_t t0 = HAL_GetTick();

        NEUQ_IMU_Calib();           /* 上电静默零偏校准 1.5 s */
        while ((NEUQ_IMU_CalibBusy() != 0U) && ((uint32_t)(HAL_GetTick() - t0) < 5000U))
        {
        }
    }
    else
    {
        NEUQ_Menu2_ShowMessage("MPU6050 NOT FOUND", "check PA8/PC9 wiring");
        NEUQ_Delay_ms(1200);
    }

    NEUQ_Menu2_Redraw();

    while (1)
    {
        uint8_t      i;
        KeyEventEnum event;

        for (i = 0U; i < (uint8_t)KEY_ID_NUM; i++)
        {
            event = NEUQ_Key_GetEvent((KeyIdEnum)i);
            if (event != KEY_EVENT_NONE)
            {
                NEUQ_Menu2_HandleKey((KeyIdEnum)i, event);
            }
        }

        NEUQ_Menu2_Task();
    }
}
