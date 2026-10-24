#include "code.h"
#include "imu.h"
#include "param.h"
#include "menu.h"
#include <stdio.h>

/* PIT 中断共享变量（NEUQ_Interrupts.c 中 extern 声明，1ms 递增） */
volatile uint32_t time_tick = 0;

static uint32_t last_sample_tick, last_display_tick;

static void VOFA_SendFrame(float raw, float flt) {
    char buf[48];
    snprintf(buf, sizeof(buf), "%.4f,%.4f\n", raw, flt);
    NEUQ_Uart_write_string(VOFA_UART, (uint8_t *)buf);
}

static void Sample_Task(void) {
    NEUQ_MPU6050_read_data();
    raw_axis[AX_AX] = MPU6050_acc_transition(mpu6050_acc_x);
    raw_axis[AX_AY] = MPU6050_acc_transition(mpu6050_acc_y);
    raw_axis[AX_AZ] = MPU6050_acc_transition(mpu6050_acc_z);
    raw_axis[AX_GX] = MPU6050_gyro_transition(mpu6050_gyro_x);
    raw_axis[AX_GY] = MPU6050_gyro_transition(mpu6050_gyro_y);
    raw_axis[AX_GZ] = MPU6050_gyro_transition(mpu6050_gyro_z);
    Bias_Track(); Filter_Update();
    Attitude_Update();
    if (param.send_enable && param.send_axis < AX_NUM)
        VOFA_SendFrame(raw_axis[param.send_axis], filt_axis[param.send_axis]);
}

/*----------------------------------------------------------------
* 函数简介     初始化外设与模块
*----------------------------------------------------------------*/
void Code_Init(void) {
    NEUQ_LCD200_init();
    NEUQ_PIT_init_ms(PIT_CH1, 1);              /* 启动 NEUQ 1ms 时基 */
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);   /* 统一全屏字号为 16px */

    IMU_Init();
    Gyro_Calibrate();

    Param_Init(); Param_Load();

    NEUQ_Uart_init(VOFA_UART, VOFA_UART_BAUD);

    last_sample_tick = last_display_tick = time_tick;
    Menu_Init();                 /* 主菜单链表连接 */
    Page_Enter(PAGE_MAIN);
}

/*----------------------------------------------------------------
* 函数简介     主循环任务
*----------------------------------------------------------------*/
void Code_Task(void) {
    uint32_t t0 = time_tick;
    uint8_t ku = Key_Read(KEY_UP), kd = Key_Read(KEY_DOWN), ke = Key_Read(KEY_ENTER), kb = Key_Read(KEY_BACK);
    if (time_tick - last_sample_tick >= SAMPLE_PERIOD_MS) {
        last_sample_tick += SAMPLE_PERIOD_MS; Sample_Task(); }
    Handle_Keys(ku, kd, ke, kb);
    if (time_tick - last_display_tick >= DISPLAY_PERIOD_MS) {
        last_display_tick += DISPLAY_PERIOD_MS; Refresh_Values(); }
    if ((int32_t)(time_tick - t0) < 1) NEUQ_Delay_ms(1 - (time_tick - t0));
}
