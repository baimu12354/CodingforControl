/*
 * NEUQ_App.c
 * 应用层入口：初始化 + 主循环分时调度
 *
 * 不用任何定时器中断，主循环里靠DWT->CYCCNT比时间，到点就干一件事：
 *   每  2000us   读MPU6050 -> 六轴滤波 -> 姿态更新 -> 按速率上传
 *   每 10000us   按键扫描
 *   每100000us   刷新当前页面，10Hz人眼看着就够流畅了
 *
 * 采样这一路里没有任何"当前在第几页"的判断，
 * 所以来回切页面不会打断采集、滤波和姿态解算。
 *
 * 姿态更新周期就是采样周期2ms，也就是500Hz，而且dt取的是两次读数之间的
 * 实测时间差，不是写死的2ms。刷屏或者写Flash偶尔卡一下的时候积分也不会算错。
 */
#include "NEUQ_lib_include.h"
#include "NEUQ_Setting.h"
#include "NEUQ_IMU.h"
#include "NEUQ_Attitude.h"
#include "NEUQ_Wave.h"
#include "NEUQ_Key.h"
#include "NEUQ_MENU.h"
#include "NEUQ_App.h"

// 三个调度周期，单位微秒
#define APP_SAMPLE_US       ((uint32_t)NEUQ_IMU_PERIOD_MS * 1000U)  // 2000us  采样
#define APP_KEY_US          ((uint32_t)KEY_SCAN_PERIOD_MS * 1000U)  // 10000us 按键
#define APP_DISP_US         (100000U)                               // 100ms   显示

// 开机零偏校准点数，500Hz下1000点就是2秒
#define APP_CAL_SAMPLES     (1000U)

static uint32_t s_last_sample_us = 0;
static uint32_t s_last_key_us    = 0;
static uint32_t s_last_disp_us   = 0;
static uint32_t s_prev_sample_us = 0;
static uint32_t s_cyc_per_us     = 168U;    // DWT计数换算成微秒的系数

// 微秒级时间戳
// CYCCNT是32位，168MHz下大约25.6秒回绕一次，所以下面所有时间比较
// 都写成 (now - last) >= 周期 的无符号减法，回绕了结果也是对的
static uint32_t NEUQ_App_Micros(void)
{
    return DWT->CYCCNT / s_cyc_per_us;
}

void NEUQ_App_Init(void)
{
    uint8_t mpu_ok;

    // 先把调试串口和延时准备好，后面初始化都要用
    NEUQ_Debug_init();
    NEUQ_Delay_init();

    s_cyc_per_us = SystemCoreClock / 1000000U;
    if(s_cyc_per_us == 0U)
    {
        s_cyc_per_us = 1U;
    }

    NEUQ_Delay_ms(50);              // 等电源和外设稳定

    NEUQ_LCD200_init();
    NEUQ_Setting_Load();            // 从Flash恢复参数，没有有效记录就用默认值
    NEUQ_Wave_Init();
    NEUQ_Key_Init();

    // 上传串口自检
    // 屏幕显示OK说明单片机这边确实发出去了，收不到就是接线或者上位机设置的问题；
    // 显示FAIL说明串口外设本身有问题，比如引脚复用被别的功能占了。
    // 自检发出去的是一帧合法的全0数据，在VOFA+上只是t=0处的一个点，不影响看波形。
    {
        uint8_t tx_ok;
        char    l1[32];
        char    l2[32];

        // 端口探针：往USART1(PA9)和UART4(PC10)各发一行带名字的文字，
        // 在电脑上看哪一行出来，就知道这个COM口通到板上哪个脚
        NEUQ_Wave_PortProbe();

        tx_ok = NEUQ_Wave_SelfTest();

#if (NEUQ_WAVE_UART == UART_0)
        snprintf(l1, sizeof(l1), "TX PORT: USART1 PA9");
        snprintf(l2, sizeof(l2), "SELF-TEST %s  115200 8N1", tx_ok ? "OK" : "FAIL");
#elif (NEUQ_WAVE_UART == UART_1)
        snprintf(l1, sizeof(l1), "TX PORT: USART2 PA2");
        snprintf(l2, sizeof(l2), "SELF-TEST %s  115200 8N1", tx_ok ? "OK" : "FAIL");
#elif (NEUQ_WAVE_UART == UART_3)
        snprintf(l1, sizeof(l1), "TX PORT: UART4  PC10");
        snprintf(l2, sizeof(l2), "SELF-TEST %s  115200 8N1", tx_ok ? "OK" : "FAIL");
#else
        snprintf(l1, sizeof(l1), "TX PORT: see Wave.h");
        snprintf(l2, sizeof(l2), "SELF-TEST %s", tx_ok ? "OK" : "FAIL");
#endif

        // 这两行也打到串口上。助手里出现这几行说明链路是通的，
        // 一片空白就不用查代码了，直接查线。
        NEUQ_Wave_Log("\r\n[BOOT] ===== 6-AXIS SYSTEM =====\r\n");
        NEUQ_Wave_Log("[BOOT] ");
        NEUQ_Wave_Log(l1);
        NEUQ_Wave_Log("  ");
        NEUQ_Wave_Log(l2);
        NEUQ_Wave_Log("\r\n");

        NEUQ_Menu_BootScreen(l1, l2);
        NEUQ_Delay_ms(800);
    }

    NEUQ_Menu_BootScreen("MPU6050 INIT...", "I2C3 PA8=SCL PC9=SDA");
    NEUQ_Delay_ms(200);

    mpu_ok = NEUQ_IMU_Init();

    if(mpu_ok == 0)
    {
        // 传感器没找到也不死机，菜单和Flash参数功能照样能用，
        // 屏幕上给出提示方便现场查接线
        NEUQ_Menu_BootScreen("MPU6050 NOT FOUND", "CHECK I2C3 PA8/PC9");
        NEUQ_Delay_ms(1500);
        g_imu.online = 0;
    }
    else
    {
        // 等内部的DLPF稳定下来再进校准
        NEUQ_Delay_ms(100);
    }

    // 陀螺静态零偏校准，这一步要模块保持静止，
    // 姿态静置不漂主要就靠它
    NEUQ_Menu_BootScreen("KEEP THE BOARD STILL", "GYRO CALIBRATING 2s");
    NEUQ_IMU_CalibrateGyro(APP_CAL_SAMPLES);

    NEUQ_IMU_SetFilterHz(g_setting.acc_lpf_hz, g_setting.gyro_lpf_hz);
    NEUQ_Attitude_SetGains(g_setting.att_kp, g_setting.att_ki);

    // 采一次真实数据，用加速度计把四元数直接摆到当前姿态。
    // 以前是让四元数从单位四元数被加速度计慢慢拉过去，那要好几秒，
    // 开机后屏幕上那三个角会一直爬，看着就像解算错了。现在一步到位。
    NEUQ_IMU_Update();
    NEUQ_Attitude_InitFromAccel(g_imu.acc_g);

    // 再空跑100ms，让滤波器和姿态都进稳态
    {
        uint16_t i;
        for(i = 0; i < 50; i++)             // 50 * 2ms = 100ms
        {
            NEUQ_IMU_Update();
            NEUQ_Attitude_Update(g_imu.acc_g, g_imu.gyro_dps,
                                 (float)NEUQ_IMU_PERIOD_MS / 1000.0f);
            NEUQ_Delay_ms(NEUQ_IMU_PERIOD_MS);
        }
    }

    // 把开机时的朝向记成航向0°。六轴没有绝对航向参考，只能给相对值
    NEUQ_Attitude_SetYawZero();

    s_prev_sample_us = s_last_sample_us = NEUQ_App_Micros();
    s_last_key_us    = s_last_sample_us;
    s_last_disp_us   = s_last_sample_us;

    NEUQ_Menu_Init();
    NEUQ_Menu_SetStatus(g_setting_valid ? "FLASH" : "DEFAULT");

    // 这里用NEUQ_Wave_Log而不是printf
    // printf底层是没有超时的死等，串口一出问题上电初始化就停在最后这一行，
    // 现象正好是"主菜单画出来了但按键全没反应"。
    // 又因为要打在"看波形的那个串口"上，所以走Wave_Log而不是Menu_DebugPrint。
    {
        char line[96];

#if (NEUQ_WAVE_UART == UART_0)
#define APP_WAVE_UART_NAME      "USART1(PA9)"
#elif (NEUQ_WAVE_UART == UART_1)
#define APP_WAVE_UART_NAME      "USART2(PA2)"
#elif (NEUQ_WAVE_UART == UART_2)
#define APP_WAVE_UART_NAME      "USART3(PB10)"
#elif (NEUQ_WAVE_UART == UART_3)
#define APP_WAVE_UART_NAME      "UART4(PC10)"
#else
#define APP_WAVE_UART_NAME      "?"
#endif

        snprintf(line, sizeof(line),
                 "[APP] init done. %s, acc_lpf=%.0fHz gyro_lpf=%.0fHz Kp=%.2f Ki=%.3f\r\n",
                 g_setting_valid ? "param from FLASH" : "param DEFAULT",
                 g_setting.acc_lpf_hz, g_setting.gyro_lpf_hz, g_setting.att_kp, g_setting.att_ki);
        NEUQ_Wave_Log(line);

        snprintf(line, sizeof(line),
                 "[APP] wave uart=%s self-test=%s wave_enable=%u\r\n",
                 APP_WAVE_UART_NAME,
                 NEUQ_Wave_IsUartOk() ? "OK" : "FAIL",
                 (unsigned)g_setting.wave_enable);
        NEUQ_Wave_Log(line);
    }
}

void NEUQ_App_Loop(void)
{
    uint32_t now = NEUQ_App_Micros();
    KEY_EVENT_ENUM ev;

    // 任务1：采样 + 滤波 + 姿态解算 + 波形上传，500Hz
    // 这一段跟当前页面无关，切页面不会中断
    if((uint32_t)(now - s_last_sample_us) >= APP_SAMPLE_US)
    {
        uint32_t t_now;
        float dt_s;

        s_last_sample_us += APP_SAMPLE_US;
        if((uint32_t)(now - s_last_sample_us) >= APP_SAMPLE_US)
        {
            s_last_sample_us = now;     // 落后太多（比如刚写完Flash），重新对齐
        }

        // 读传感器 + 六轴独立低通滤波
        NEUQ_IMU_Update();

        // 姿态解算，dt用两次读数之间的实测差，不是写死的2ms
        t_now  = NEUQ_App_Micros();
        dt_s   = (float)(uint32_t)(t_now - s_prev_sample_us) * 0.000001f;
        s_prev_sample_us = t_now;

        NEUQ_Attitude_Update(g_imu.acc_g, g_imu.gyro_dps, dt_s);

        // 上传，只有开了发送才真的发数据，关掉的时候上面两步照常执行
        NEUQ_Wave_Tick();
    }

    // 任务2：按键扫描，10ms一次
    now = NEUQ_App_Micros();
    if((uint32_t)(now - s_last_key_us) >= APP_KEY_US)
    {
        s_last_key_us += APP_KEY_US;
        if((uint32_t)(now - s_last_key_us) >= APP_KEY_US)
        {
            s_last_key_us = now;
        }
        NEUQ_Key_Scan();
    }

    // 把这一轮攒下的按键事件全处理掉。长按会连续产生事件，所以要用while
    while((ev = NEUQ_Key_GetEvent()) != KEY_EVENT_NONE)
    {
        NEUQ_Menu_HandleKey(ev);
    }

    // 任务3：刷新当前页面，10Hz
    now = NEUQ_App_Micros();
    if((uint32_t)(now - s_last_disp_us) >= APP_DISP_US)
    {
        s_last_disp_us += APP_DISP_US;
        if((uint32_t)(now - s_last_disp_us) >= APP_DISP_US)
        {
            s_last_disp_us = now;
        }
        NEUQ_Menu_Update();
    }
}
