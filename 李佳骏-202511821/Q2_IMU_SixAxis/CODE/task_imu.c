/* task_imu.c - 第二题任务入口 */

#include "task_imu.h"
#include "board_config.h"
#include "app_tick.h"
#include "bsp_key.h"
#include "imu_mpu6050.h"
#include "imu_filter.h"
#include "imu_attitude.h"
#include "menu.h"
#include "param_store.h"
#include "ui_imu.h"
#include "vofa.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

static imu_view_t s_view;

void Task_IMU_Init(void)
{
    uint8_t i;

    NEUQ_Debug_init();
    NEUQ_Delay_init();
    NEUQ_LCD200_init();

    NEUQ_Uart_init(UART_0, UART_BAUD_115200);
    NEUQ_Uart_init(UART_3, UART_BAUD_115200);

    Key_Init();
    AppTick_Init();
    Menu_Init();
    ImuFilter_Init();
    /* 姿态解算：初始化并开始零偏校准，PIT 节拍随后启动 */
    Attitude_Init();

    /* 先把界面数据清干净，避免传感器还没读出数时显示随机值 */
    s_view.raw.acc[0] = 0.0f; s_view.raw.acc[1] = 0.0f; s_view.raw.acc[2] = 0.0f;
    s_view.raw.gyro[0] = 0.0f; s_view.raw.gyro[1] = 0.0f; s_view.raw.gyro[2] = 0.0f;
    s_view.raw.temp_c = 0.0f;
    for (i = 0; i < IMU_AXIS_COUNT; i++)
    {
        s_view.filtered[i] = 0.0f;
    }

    /* 识别失败也继续跑，屏幕显示 not found */
    s_view.imu_ok    = IMU_Init();
    s_view.imu_addr7 = IMU_GetAddress();

    /* 从 Flash 恢复上次保存的滤波系数；没有有效记录就保持默认值 */
    (void)ParamStore_LoadAndApply();

    Ui_Imu_DrawPage(&s_view);

    NEUQ_PIT_init_ms(PIT_CH1, T2_TICK_MS);      /* 启动 1 ms 节拍 */
}

static void Task_IMU_Sample(void)
{
    imu_sample_t raw;

    if (IMU_Read(&raw) != 0U)
    {
        s_view.raw = raw;
        ImuFilter_Update(&raw, s_view.filtered);

        /* 姿态解算跟着每一次有效采样走，和当前在哪个页面无关，
         * 这样「切换页面时继续采集与解算」是天然成立的。 */
        Attitude_Update(&raw, AppTick_GetMs());
    }
}

static void Task_IMU_Upload(void)
{
    float      pair[2];
    imu_axis_t axis = Menu_GetAxis();

    pair[0] = IMU_AxisValue(&s_view.raw, axis);
    pair[1] = s_view.filtered[axis];

    Vofa_SendFloats(pair, 2U);
}

void Task_IMU_Loop(void)
{
    /* 1. 采样与滤波：不受页面和上传开关影响 */
    if (AppTick_TakeFlag(APP_FLAG_SAMPLE))
    {
        Task_IMU_Sample();
    }

    /* 2. 按键与页面跳转 */
    if (Menu_HandleKey() != 0U)
    {
        Ui_Imu_DrawPage(&s_view);
    }

    /* 3. 保存成功的提示 */
    if (Menu_TakeSavedFlag() != 0U)
    {
        Ui_Imu_ShowSaved();
    }

    /* 4. 上传：只有开着才发，关掉只是不发，采集与滤波照旧 */
    if (AppTick_TakeFlag(APP_FLAG_UPLOAD))
    {
        if (Menu_IsUploadOn() != 0U)
        {
            Task_IMU_Upload();
        }
    }

    /* 5. 数值刷新 */
    if (AppTick_TakeFlag(APP_FLAG_UI))
    {
        Ui_Imu_RefreshValues(&s_view);
    }
}
