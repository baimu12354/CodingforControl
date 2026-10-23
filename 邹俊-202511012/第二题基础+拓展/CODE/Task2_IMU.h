/*********************************************************************************************************************
* 文件名称          Task2_IMU
* 功能说明          第二题：MPU6050 六轴采集、滤波、VOFA+ 单轴上传、Flash 保存滤波参数、姿态解算，以及主函数
*
*                   资源分配（谁在什么时候做什么）：
*                   TIM6 定时中断（PIT_CH1），每 5 ms 一次，优先级 2：
*                       读 MPU6050 → 换算成 g / dps → 六个轴各自低通滤波 → 上传开启时把选中轴的 [原始, 滤波] 放进发送缓冲
*                       → 零偏校准 或 姿态更新 → 每 2 次（10 ms）扫描一次按键
*                   SysTick，优先级 0（最高）：HAL 的毫秒计时，中断里的 IIC 函数靠它判断超时
*                   主循环：按键 → 菜单；每 100 ms 刷新屏幕；把发送缓冲写到串口；保存 Flash；传感器掉线后重连
*                   菜单换页只影响显示，采集、滤波、姿态解算一直在中断里运行
*
*                   后台状态都放在全局结构体 g_t2 里：菜单直接读它来显示，改设置也直接写它，
*                   所以屏幕上显示的就是后台真正在用的值
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#ifndef _TASK2_IMU_H_
#define _TASK2_IMU_H_

#include <stdint.h>
#include "Task2_Attitude.h"

#define T2_AXIS_NUM     (6U)        // 六个轴，数组下标顺序：0 AX  1 AY  2 AZ  3 GX  4 GY  5 GZ

typedef struct {
    // ---------------- 采样结果：中断写，菜单只读 ----------------
    int16_t  raw[T2_AXIS_NUM];          // 原始计数
    float    value[T2_AXIS_NUM];        // 换算后的物理量：加速度 g，角速度 dps
    float    filtered[T2_AXIS_NUM];     // 低通滤波后的物理量
    float    temperature;               // 芯片温度 ℃
    uint32_t sample_count;              // 成功采样的次数

    // ---------------- 设置：菜单写，中断读 ----------------
    volatile uint8_t axis;              // 上传到 VOFA+ 的轴（0 ~ 5）
    volatile uint8_t upload;            // 1 表示正在上传
    float    cutoff_hz[T2_AXIS_NUM];    // 各轴低通截止频率 Hz（修改请用 T2_Imu_set_cutoff，才会同时更新滤波系数）

    // ---------------- 状态 ----------------
    volatile uint8_t sensor_ok;         // MPU6050 在线
    uint8_t  flash_ok;                  // 检测到板载 W25Q16
    uint8_t  flash_has_record;          // Flash 里有有效的参数记录（上电读到了，或者保存过）
    uint8_t  param_dirty;               // 参数改过但还没保存
    uint8_t  save_failed;               // 最近一次保存失败

    // ---------------- 请求：菜单置 1，中断里执行完清 0 ----------------
    // 姿态和校准数据只在中断里修改，主循环想改就"提请求"，这样两边永远不会同时改同一份数据
    volatile uint8_t calib_request;     // 重新做零偏校准
    volatile uint8_t zero_yaw_request;  // 航向清零
} T2_Imu_Struct;

extern T2_Imu_Struct      g_t2;             // 后台状态
extern T2_Attitude_Struct g_t2_attitude;    // 姿态（中断写，菜单读）
extern T2_Calib_Struct    g_t2_calib;       // 零偏校准（中断写，菜单读）

void    Task2_Main(void);                               // 第二题主函数（不返回）
void    Task2_PIT_Handler(void);                        // 5 ms 定时中断处理函数，在 NEUQ_PIT_1_CALLBACK 中调用
void    T2_Imu_set_cutoff(uint8_t axis, float cutoff_hz);   // 修改某个轴的截止频率，立即生效
uint8_t T2_Imu_save_param(void);                        // 把截止频率保存到 Flash，成功返回 1（耗时几十 ms）

#endif
