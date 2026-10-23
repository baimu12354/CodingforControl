#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>

/* 输入：模块右手坐标系的 ax/ay/az (g)、gx/gy/gz (deg/s)。
 * 水平基准姿态的加速度为 (0,0,+1g)，正旋转遵循右手定则。
 * 输出：ZYX 欧拉角，roll 绕 X、pitch 绕 Y、yaw 绕 Z，单位 deg。
 * yaw 以校准完成时为零点；六轴传感器没有绝对航向参考。
 */
#define ATTITUDE_CALIBRATION_SAMPLES 300U
#define ATTITUDE_WARMUP_SAMPLES       50U

typedef enum {
    ATTITUDE_CALIBRATING = 0,
    ATTITUDE_READY,
    ATTITUDE_SENSOR_ERROR
} Attitude_Status;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float q[4];             /* 标量在前，表示模块坐标系到参考系的旋转 */
    float gyro_bias[3];     /* deg/s，与输入陀螺仪数据单位相同 */
    float dt_ms;
    uint16_t calibration_count;
    Attitude_Status status;
} Attitude_Data;

/* Init/ProcessSample/OnSensorError 由采样侧调用，不能互相抢占。 */
void Attitude_Init(void);
void Attitude_ProcessSample(const float data[6], float dt_seconds);
void Attitude_OnSensorError(void);

/* 菜单只提交请求；在下一次采样中重新校准，不在菜单中修改四元数。 */
void Attitude_RequestCalibration(void);

/* 复制快照。若采样在中断、读取在主循环，调用方须短暂屏蔽采样中断。 */
void Attitude_GetData(Attitude_Data *out);

#endif
