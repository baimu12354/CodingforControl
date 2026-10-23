#ifndef NEUQ_ATTITUDE_H_
#define NEUQ_ATTITUDE_H_

// 姿态角（度）
extern float attitude_roll;
extern float attitude_pitch;
extern float attitude_yaw;
extern float gyro_bias[3];   // 三轴陀螺零偏(°/s)，NEUQ_Sensor 也用它做零偏校正

void NEUQ_Attitude_Init(void);
void NEUQ_Attitude_Task(void);
void NEUQ_Attitude_Calibrate(void);   // 手动重标定零偏并保存（菜单 GYRO CAL 触发）

#endif
