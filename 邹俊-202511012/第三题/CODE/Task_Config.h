/*********************************************************************************************************************
* 文件名称          Task_Config
* 功能说明          第三题 串口调参与电机闭环控制：任务选择 + 板级引脚/参数配置
*                   换板子时只需要修改本文件，其余代码不需要改动
* 适用平台          STM32F407VE（NEUQ 开源库）
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#ifndef _TASK_CONFIG_H_
#define _TASK_CONFIG_H_

//====================================================================================================================
// 一、本分支只包含第三题 串口调参与电机闭环控制，TASK_SELECT 固定为 3（代码中用它区分题目，不要修改）
//====================================================================================================================
#define TASK_SELECT                     (3)

//====================================================================================================================
// 五、第三题：串口字节包调参 + 电机速度闭环（不用菜单、LCD、Flash）
//     本台架的实测接法：电机动力线在驱动板通道2（PA2 给 PWM_2_IN、PA5 给 DIR_2_IN），
//     编码器插头在通道1 的座（PB6 / PB7），上位机接核心板 PC10 / PC11（UART_3）；调试串口仍为 PA9 / PA10。
//     驱动板 DRV8701E 使用 PH/EN：方向 + PWM；SW1 必须使能 nSLEEP
//====================================================================================================================
#define T3_UART_INDEX                   (3)             // 上位机串口：0 为 UART_0(PA9/PA10)，1 为 UART_1(PA2/PA3)，
                                                        //             2 为 UART_2(PB10/PB11)，3 为 UART_3(PC10/PC11)
#define T3_UART_BAUD                    (UART_BAUD_115200)
#define T3_UART_RX_PRIORITY             (3)             // 串口接收中断优先级（数字越大越不优先，比控制中断低）
#define T3_CONTROL_PERIOD_MS            (5)             // 控制周期：每 5 ms 测一次速、算一次 PID、更新一次 PWM
#define T3_TELEMETRY_DIVIDER            (2)             // 每 2 个控制周期回传一次（10 ms）

#define T3_MOTOR_PWM                    (TIM2_CH3_PA2)  // H11-10 -> PWM_2_IN -> EN_2（电机动力线在通道2）
#define T3_MOTOR_DIR                    (PA5)           // H11-2  -> DIR_2_IN -> PH_2
#define T3_MOTOR_FORWARD_LEVEL          (GPIO_High)     // 正输出时的方向电平；需与编码器正方向一致
#define T3_MOTOR_REVERSE_LEVEL          (GPIO_Low)
#define T3_MOTOR_PWM_FREQ_HZ            (8000)          // PWM 频率（库的上限约 8.4 kHz）
#define T3_PWM_LIMIT                    (9500.0f)       // PWM 输出最大值（满量程 10000，即最大 95%）

#define T3_ENCODER_TIM                  (Encoder_TIM4)  // 编码器插头在通道1的座：H11-7/H11-3 -> PB6/PB7
#define T3_ENCODER_PHASE_A              (Encoder_TIM4_PhaseA_PB6)
#define T3_ENCODER_PHASE_B              (Encoder_TIM4_PhaseB_PB7)
#define T3_ENCODER_LINES                (11.0f)         // 编码器线数（电机转一圈一相输出多少个脉冲）：JGB37-520 配的磁性霍尔编码器 11PPR，装在电机高速轴上
#define T3_ENCODER_MULTIPLE             (4.0f)          // 定时器编码器模式对 A、B 两相的上升沿和下降沿都计数：4 倍
#define T3_GEAR_RATIO                   (10.0f)         // 减速比，JGB37-520 为 1:10，直驱电机为 1
#define T3_ENCODER_DIR_INVERT           (1)             // 电机正转时测出的转速是负数，就改成 1
#define T3_SPEED_FILTER_ALPHA           (0.5f)          // 测速滤波系数（1 表示不滤波）

#define T3_TARGET_LIMIT_RPM             (900.0f)        // 目标转速的最大值（超过会被限制在这个值）：实测占空比约 0.6% + 0.102%×rpm，95% 限幅时最高约 925 rpm
#define T3_DEFAULT_KP                   (2.0f)          // 上电默认的 PID 参数（误差单位 rpm，输出单位 PWM 计数）
#define T3_DEFAULT_KI                   (0.12f)
#define T3_DEFAULT_KD                   (0.0f)
#define T3_PID_PARAM_MAX                (655.35f)       // 上位机发来的 PID 参数超过这个值会被拒绝

// 方向异常保护：目标和输出都与转速反向，且输出接近限幅、转速超过100 rpm，
// 连续1秒没有明显减速就停机。错误方向稳定高速也算异常，不要求速度一直增长。
#define T3_FAULT_SPEED_RPM              (100.0f)
#define T3_FAULT_DECEL_RPM              (5.0f)          // 低于约100ms慢速基准超过此值，认为正在明显减速
#define T3_FAULT_TIME_MS                (1000U)

#endif
