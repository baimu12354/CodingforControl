#ifndef IMU_STATE_H
#define IMU_STATE_H

/* CODE 内部共享状态：各变量由所属功能模块唯一定义；主循环通过函数访问。 */
#include "Imu_Config.h"

extern AppConfig g_config;
extern ImuData g_imu;
extern uint8_t g_mpu_ok;

#endif /* IMU_STATE_H */
