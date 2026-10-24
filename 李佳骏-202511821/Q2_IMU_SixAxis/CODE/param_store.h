/* param_store.h - 参数存储接口 */

#ifndef _PARAM_STORE_H_
#define _PARAM_STORE_H_

#include "stm32f4xx_hal.h"
#include "imu_mpu6050.h"

/* 保存在 Flash 里的内容 */
typedef struct {
    uint32_t magic;                        /* 固定为 T2_PARAM_MAGIC 才认为有效 */
    float    alpha[IMU_AXIS_COUNT];        /* 六个轴各自的滤波系数 */
    uint32_t checksum;                     /* 对前面所有字节求和，用于发现数据损坏 */
} imu_params_t;

/* 初始化 Flash 并尝试读回参数。读回有效时自动应用到滤波器上。
 * 返回 1 = 用上了 Flash 里的参数，0 = 没有有效记录，正在用默认值。 */
uint8_t ParamStore_LoadAndApply(void);

/* 把当前滤波器里的系数存进 Flash。返回 1 = 成功 */
uint8_t ParamStore_SaveCurrent(void);

#endif /* _PARAM_STORE_H_ */
