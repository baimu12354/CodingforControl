/* ui_imu.h - 第二题页面接口 */

#ifndef _UI_IMU_H_
#define _UI_IMU_H_

#include "stm32f4xx_hal.h"
#include "imu_mpu6050.h"

/* 界面要显示的一帧数据 */
typedef struct {
    imu_sample_t raw;                       /* 最新一次原始采样 */
    float        filtered[IMU_AXIS_COUNT];  /* 对应的滤波结果 */
    uint8_t      imu_ok;                    /* 传感器是否初始化成功 */
    uint8_t      imu_addr7;                 /* 探测到的 7 位地址 */
} imu_view_t;

void Ui_Imu_DrawPage(const imu_view_t *view);        /* 整页重画 */
void Ui_Imu_RefreshValues(const imu_view_t *view);   /* 只刷新数值 */

/* 在页面底部提示一次「参数已写入 Flash」，下次整页重画时消失 */
void Ui_Imu_ShowSaved(void);

#endif /* _UI_IMU_H_ */
