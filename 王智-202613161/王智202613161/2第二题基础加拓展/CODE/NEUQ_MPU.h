#ifndef _NEUQ_MPU_H_
#define _NEUQ_MPU_H_
#define DEG2RAD                   (0.017453292519943295f)   // 度 -> 弧度
#define RAD2DEG                   (57.29577951308232f)      // 弧度 -> 度
typedef struct{
    uint32_t magic;     // 校验头，读对才算有效，否则用默认值
    int32_t  param;     // 滤波参数 1~100，对应 alpha = param/100.0f
} NEUQ_MPU_FilterConfig;

void  NEUQ_MPU_Init(void);                        // 上电初始化：Flash 恢复参数 + 四元数初始化 + 陀螺零偏校准
void  NEUQ_MPU_Task(void);                        // 后台任务：512Hz 采样/滤波/姿态/上传（主循环每圈调用）
void  NEUQ_MPU_SaveParam(void);                   // 把当前滤波参数写入 Flash（菜单「保存」时调用）
float NEUQ_MPU_GetRawValue(uint8_t axis);         // 取指定轴原始值  1=Ax 2=Ay 3=Az 4=Gx 5=Gy 6=Gz
float NEUQ_MPU_GetFilteredValue(uint8_t axis);    // 取指定轴滤波值
void  NEUQ_MPU_ShowAttitude(void);                // LCD 显示横滚/俯仰/航向角（姿态页渲染时调用）
float NEUQ_MPU_GetCalibratedGyro(uint8_t axis);  // 取陀螺去零偏值（1=Gx 2=Gy 3=Gz）
#endif