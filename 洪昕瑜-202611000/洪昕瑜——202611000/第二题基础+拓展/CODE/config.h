/*********************************************************************************************************************
* 文件名称          config
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
********************************************************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

// ===== 串口 / 上位机 =====
#define VOFA_UART          UART_0                          // 上位机(VOFA)串口
#define VOFA_UART_BAUD     UART_BAUD_115200                // 串口波特率

// ===== 采样 / 显示周期 =====
#define SAMPLE_PERIOD_MS   5                               // 采样周期(ms)，200Hz
#define SAMPLE_DT          (SAMPLE_PERIOD_MS / 1000.0f)    // 采样周期(s)，由 SAMPLE_PERIOD_MS 派生
#define DISPLAY_PERIOD_MS  100                             // 显示刷新周期(ms)

// ===== 按键 =====
#define KEY_UP_PIN         PE10                            // 上
#define KEY_DOWN_PIN       PE8                             // 下
#define KEY_ENTER_PIN      PE7                             // 确认
#define KEY_BACK_PIN       PE9                             // 返回
#define KEY_ACTIVE_LEVEL   GPIO_Low                        // 按下有效电平

// ===== IMU 滤波 / 姿态 =====
#define STILL_DPS          1.0f                            // 静止判定阈值
#define BIAS_K             0.005f                          // 零偏收敛系数
#define GYRO_SCALE_K       1.034f                          // 陀螺比例修正
#define MAHONY_KP          2.0f                            // 姿态互补滤波 P 系数

// ===== Flash 参数存储 =====
#define PARAM_SECTOR       NEUQ_FLASH_LAST_SECTOR_ID       // 参数存储扇区
#define PARAM_MAGIC        0x4E455551UL                    // 参数校验魔数

#endif
