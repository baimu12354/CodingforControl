/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_Config
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
*
* 说明：硬件接线、控制参数、通信协议都集中在这里，换板子/换电机只改本文件。
********************************************************************************************************************/
#ifndef _NEUQ_CONFIG_H_
#define _NEUQ_CONFIG_H_

#include "NEUQ_lib_include.h"

/*===================================================================================================================
 * 引脚分配（按主板 H11 接插件与驱动板 2×5 信号座的实际连线）
 *
 *   H11 脚   MCU 脚   功能
 *   ------   ------   ------------------------------------------
 *    1       PC7     电机1 方向  -> DRV8701 PH
 *    2       PA5     电机2 方向  -> DRV8701 PH
 *    3       PB7     电机1 编码器 A（TIM4_CH2）
 *    4       PA7     电机2 编码器 A（TIM3_CH2）
 *    7       PB6     电机1 编码器 B（TIM4_CH1）
 *    8       PA6     电机2 编码器 B（TIM3_CH1）
 *    9       PC6     电机1 转速  -> DRV8701 EN
 *   10       PA2     电机2 转速  -> DRV8701 EN
 *
 *   串口 USART1：PA9(TX) / PA10(RX)，接 ST-Link 虚拟串口
 *==================================================================================================================*/

/*----------------------------------------- 1. 上位机通信 ----------------------------------------------*/
#define NEUQ_COMM_UART                UART_0
#define NEUQ_COMM_UART_INDEX          0                   // 0=UART_0 1=UART_1 2=UART_2 3=UART_3
#define NEUQ_COMM_BAUD                UART_BAUD_115200    // ST-Link 虚拟串口 115200 最稳
#define NEUQ_COMM_RX_IRQ_PRIO         5                   // 接收中断优先级，必须比控制中断高

/*----------------------------------------- 2. 控制节拍 ------------------------------------------------*/
#define NEUQ_PIT_CHANNEL              PIT_CH1             // TIM6
#define NEUQ_PIT_CHANNEL_INDEX        1                   // 1=PIT_CH1(TIM6) 2=PIT_CH2(TIM7)
#define NEUQ_CONTROL_PERIOD_MS        5                   // 5ms = 200Hz
#define NEUQ_CONTROL_DT               (NEUQ_CONTROL_PERIOD_MS / 1000.0f)
#define NEUQ_CONTROL_IRQ_PRIO         6                   // 低于串口接收，保证收字节不被打断

/*----------------------------------------- 3. 编码器 --------------------------------------------------*/
/* 两对脚都支持，运行时自动识别（见 NEUQ_Speed.c）：
 *   电机1：PB6/PB7 = TIM4
 *   电机2：PA6/PA7 = TIM3
 * 换过驱动板，编码器实际接在哪一对不确定，所以两个都初始化。 */
#define NEUQ_ENCODER_DEV              Encoder_TIM4
#define NEUQ_ENCODER_PHASE_A          Encoder_TIM4_PhaseA_PB6
#define NEUQ_ENCODER_PHASE_B          Encoder_TIM4_PhaseB_PB7
#define NEUQ_ENC_PULLUP_GPIO          GPIOB
#define NEUQ_ENC_PULLUP_PIN           (GPIO_PIN_6 | GPIO_PIN_7)
#define NEUQ_ENC_PULLUP_AF            GPIO_AF2_TIM4

#define NEUQ_ENCODER_DIR_SIGN         (+1.0f)             // 测速方向，反了改 -1.0f（也可用 0x14 命令在线改）
#define NEUQ_ENCODER_PPR              (13.0f)             // 编码器线数（电机轴每转脉冲）
#define NEUQ_ENCODER_MULT             (4.0f)              // 库用 TI12 四倍频
#define NEUQ_ENCODER_GEAR             (30.0f)             // 减速比

/* 每转计数 = 13 × 4 × 30 = 1560。
 * 之前照另一台电机的 440 填过，测速虚高 3.5 倍，PID 一直以为超速不出力，改回来才对。 */

#define NEUQ_SPEED_FILTER_ALPHA       (0.25f)             // 一阶低通，1.0 = 不滤波

/*----------------------------------------- 4. 电机驱动 ------------------------------------------------*/
#define NEUQ_MOTOR_MODE_PWM_DIR       1                   // 单 PWM + 双方向脚（TB6612 / L298N）
#define NEUQ_MOTOR_MODE_DUAL_PWM      2                   // 双 PWM（DRV8833）
#define NEUQ_MOTOR_MODE_PH_EN         3                   // 单 PWM + 单方向脚（DRV8701）
#define NEUQ_MOTOR_MODE               NEUQ_MOTOR_MODE_PH_EN

#define NEUQ_MOTOR_PWM_PORT           TIM2_CH3_PA2       // PWM / EN
#define NEUQ_MOTOR_PWM2_PORT          TIM2_CH1_PA5       // 仅双 PWM 模式用
#define NEUQ_MOTOR_PWM_FREQ           8400               // Hz，库内 PWM_MAX = 10000
#define NEUQ_MOTOR_DIR1_PORT          PA5                // PH，原来的板子是 PA4，换了驱动板改成 PA5
#define NEUQ_MOTOR_DIR2_PORT          PB13               // 仅双方向脚驱动才用
#define NEUQ_MOTOR_DIR_SIGN           (+1.0f)             // 转向反了改 -1.0f
#define NEUQ_STOP_BRAKE               0                   // PH/EN 模式刹不住，保持 0
#define NEUQ_MOTOR_PWM_MAX            (9000.0f)           // 输出限幅（PWM_MAX = 10000）
#define NEUQ_MOTOR_ALLOW_REVERSE      0                   // 只正转：负输出一律按 0 处理

/*----------------------------------------- 5. PID -----------------------------------------------------*/
#define NEUQ_PID_KP_DEFAULT           (20.0f)
#define NEUQ_PID_KI_DEFAULT           (3.0f)              // 每拍累加，约 0.2~0.5s 消静差
#define NEUQ_PID_KD_DEFAULT           (0.0f)              // 测速是 5ms 单窗口，微分吃噪声，默认关
#define NEUQ_PID_KFF_DEFAULT          (0.0f)              // 速度前馈，调好可给 20~40
#define NEUQ_PID_I_LIMIT_DEFAULT      (20000.0f)          // Σe 限幅
#define NEUQ_PID_SLEW_DEFAULT         (0.0f)              // 每拍输出最大变化量，0 = 不限

/*----------------------------------------- 6. 保护与目标范围 ------------------------------------------*/
#define NEUQ_RUNAWAY_GUARD            0                   // 失控保护，暂时关掉
#define NEUQ_RUNAWAY_TICKS            100                 // 0.5s
#define NEUQ_RUNAWAY_TICKS2           400                 // 2s
#define NEUQ_RUNAWAY_MAX_FLIP         1

#define NEUQ_TARGET_MAX               (300.0f)
#define NEUQ_TARGET_MIN               (-300.0f)
#define NEUQ_TARGET_RAMP_ENABLE       0
#define NEUQ_TARGET_RAMP_RATE         (600.0f)            // RPM/s，仅斜坡开启时有效

/*----------------------------------------- 7. 通信协议 ------------------------------------------------*/
#define NEUQ_FRAME_HEAD0              0xAA
#define NEUQ_FRAME_HEAD1              0x55
#define NEUQ_FRAME2_HEAD0             0xCC
#define NEUQ_FRAME2_HEAD1             0xDD
#define NEUQ_FRAME_MAX_DATA           16

#define NEUQ_CMD_SET_PID              0x01
#define NEUQ_CMD_SET_TARGET           0x02
#define NEUQ_CMD_START                0x03
#define NEUQ_CMD_STOP                 0x04
#define NEUQ_CMD_SET_GAIN2            0x05
#define NEUQ_CMD_CLEAR                0x06
#define NEUQ_CMD_QUERY                0x07
#define NEUQ_CMD_OPENLOOP             0x08
#define NEUQ_CMD_SET_KP_ONE           0x11
#define NEUQ_CMD_SET_KI_ONE           0x12
#define NEUQ_CMD_SET_KD_ONE           0x13
#define NEUQ_CMD_SET_DIRSIGN          0x14
#define NEUQ_CMD_FEEDBACK             0x81                // 回传帧命令字

#define NEUQ_RX_BUF_SIZE              2048                // 必须是 2 的幂
#define NEUQ_FRAME_TIMEOUT_MS         50                  // 半帧超时

/*----------------------------------------- 8. 回传 --------------------------------------------------*/
#define NEUQ_UPLINK_MODE_VOFA         0
#define NEUQ_UPLINK_MODE_FRAME        1
#define NEUQ_UPLINK_MODE_FIREWATER    2
#define NEUQ_UPLINK_MODE              NEUQ_UPLINK_MODE_FIREWATER
#define NEUQ_UPLINK_WITH_STATE        1                   // 1 = 末尾追加 [输出, 使能] 两通道

#if (NEUQ_UPLINK_MODE == NEUQ_UPLINK_MODE_FIREWATER)
    #define NEUQ_UPLOAD_EVERY_N_TICKS 4                   // 文本帧长，4 拍发一次 = 50Hz
#else
    #define NEUQ_UPLOAD_EVERY_N_TICKS 2                   // 二进制 100Hz
#endif

#if (NEUQ_UPLINK_WITH_STATE)
    #define NEUQ_UPLINK_CH_NUM        7
#else
    #define NEUQ_UPLINK_CH_NUM        5
#endif

#endif /* _NEUQ_CONFIG_H_ */
