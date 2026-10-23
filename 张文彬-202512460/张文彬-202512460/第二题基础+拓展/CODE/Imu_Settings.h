#ifndef IMU_SETTINGS_H
#define IMU_SETTINGS_H

/* 参数默认值、校验和及 Flash 读写，保留原存储布局。 */
#include <stdint.h>
void Imu_Settings_Load(void);
uint8_t Imu_Settings_Save(void);

#endif /* IMU_SETTINGS_H */
