/* imu_attitude.h - 姿态解算接口 */

#ifndef _IMU_ATTITUDE_H_
#define _IMU_ATTITUDE_H_

#include "stm32f4xx_hal.h"
#include "imu_mpu6050.h"

/* 四元数互补滤波：陀螺仪积分 + 加速度计重力修正，输出 ZYX 欧拉角 */
typedef enum {
    ATT_STATE_WAIT = 0,         /* 准备工作，等板子放稳 */
    ATT_STATE_SAMPLING,         /* 正在采零偏 */
    ATT_STATE_READY             /* 校准完成，正常解算 */
} attitude_state_t;

typedef struct {
    float           roll;           /* 横滚角，度 */
    float           pitch;          /* 俯仰角，度 */
    float           yaw;            /* 航向角，度，相对校准完成时刻 */
    float           q[4];           /* 四元数，顺序 w, x, y, z */
    attitude_state_t state;
    uint8_t         progress;       /* 校准进度 0~100 */
    uint32_t        period_ms;      /* 上一次解算的实际间隔，毫秒 */
    float           gyro_bias[3];   /* 校准得到的零偏，单位 °/s，按芯片轴存 */
    uint32_t        updates;        /* 累计解算次数 */
} attitude_t;

void              Attitude_Init(void);
void              Attitude_StartCalibration(void);

/* 每采到一帧有效数据就调用一次。校准阶段和正常解算阶段都走这个入口。 */
void              Attitude_Update(const imu_sample_t *sample, uint32_t now_ms);

const attitude_t *Attitude_Get(void);

#endif /* _IMU_ATTITUDE_H_ */
