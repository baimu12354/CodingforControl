/*********************************************************************************************************************
* 文件名称   config
* 功能说明   参数结构体 + 全局数据 + Flash 掉电保存
* 使用方法   Config_Load() 开机调用；主循环检测 g_save_flag 后调 Config_Save()
*********************************************************************************************************************/
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "NEUQ_lib_include.h"

// 需要掉电保存的参数本体。
// 所有可编辑参数统一用 int32_t，菜单引擎只处理整数（浮点用定点放大表示）。
typedef struct {
    uint32_t magic;          // 有效性标记，判断Flash里是否有有效记录
    int32_t  send_enable;    // 波形发送启停：0=停 1=发
    int32_t  selected_axis;  // 当前发送轴：0~5
    int32_t  alpha_x1000;    // 低通系数×1000，0~1000（实际系数=该值/1000.0f）
} FilterConfig_t;

// 全局参数：菜单编辑、滤波、上传都读它（唯一数据源）
extern FilterConfig_t g_cfg;

// 六轴原始值：由MPU6050中断采集代码填充，菜单“六轴显示”页读取
extern int16_t g_acc[3];    // 三轴加速度原始值
extern int16_t g_gyro[3];   // 三轴角速度原始值
// 六轴物理值：加速度(g)、角速度(rad/s)，由中断采集代码换算，供“六轴显示”页读取
extern float g_acc_g[3];     // 三轴加速度，单位 g
extern float g_gyro_rad[3];  // 三轴角速度，单位 rad/s
// 三轴陀螺仪零偏（单位°/s）：开机静置校准得到，后续解算时从角速度里减去
extern float g_gyro_bias[3];
// 姿态角（单位度）：由 Madgwick 解算更新，菜单“姿态解算”页读取
extern float g_roll, g_pitch, g_yaw;
// 六轴滤波值：由同一中断采集代码更新，供波形对比页(VOFA)使用
extern float g_acc_f[3];    // 三轴加速度滤波值
extern float g_gyro_f[3];   // 三轴角速度滤波值

// 保存标志：菜单里按“确定”后被置1；主循环检测到为1就调 Config_Save() 再清0
extern volatile uint8_t g_save_flag;
// 上传标志：PIT_CH2定时器中断置1；主循环检测到为1就调 VOFA_Upload() 再清0
extern volatile uint8_t g_upload_tick;

// 开机调用：从Flash读参数，无有效记录则填默认值
void Config_Load(void);
// 把 g_cfg 写入Flash；成功返回NEUQ_FLASH_OK，失败返回NEUQ_FLASH_ERROR
uint8_t Config_Save(void);

#endif
