/* board_config.h - 第三题引脚与参数 */

#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

/* 当前使用 MOTOR1：PWM=PC6(TIM8_CH1)，DIR=PC7，编码器=PB6/PB7(TIM4) */
#define T3_MOTOR_CHANNEL        (1)

#if (T3_MOTOR_CHANNEL == 2)
/* MOTOR2：PWM=PA2(TIM2_CH3)，DIR=PA5，编码器=PA6/PA7(TIM3) */
#define T3_MOTOR_PWM_PORT       (TIM2_CH3_PA2)      /* → 驱动板 PWM_2_IN */
#define T3_MOTOR_DIR_PIN        (PA5)               /* → 驱动板 DIR_2_IN */
#define T3_MOTOR_PWM_USE_TIM8   (0)
#define T3_ENCODER_DEV          (Encoder_TIM3)
#define T3_ENCODER_PHASE_A      (Encoder_TIM3_PhaseA_PA6)
#define T3_ENCODER_PHASE_B      (Encoder_TIM3_PhaseB_PA7)
#else
/* MOTOR1：PWM=PC6(TIM8_CH1)，DIR=PC7，编码器=PB6/PB7(TIM4) */
#define T3_MOTOR_PWM_PIN        (PC6)               /* → 驱动板 PWM_1_IN */
#define T3_MOTOR_DIR_PIN        (PC7)               /* → 驱动板 DIR_1_IN */
#define T3_MOTOR_PWM_USE_TIM8   (1)
#define T3_ENCODER_DEV          (Encoder_TIM4)
#define T3_ENCODER_PHASE_A      (Encoder_TIM4_PhaseA_PB6)
#define T3_ENCODER_PHASE_B      (Encoder_TIM4_PhaseB_PB7)
#endif

/* 本题不用 Flash，片选拉高避免 SPI 引脚浮空 */
#define T3_FLASH_CS_PIN         (PB0)

/* 库按整数分频，实际输出约 8400 Hz */
#define T3_PWM_FREQ_HZ          (8000U)

/* 实测每转计数：11 线 × 4 倍频 × 10 减速比 = 440 */
#define T3_ENCODER_COUNT_PER_REV    (440.0f)

/* 方向取反时改符号 */
#define T3_ENCODER_DIRECTION    (+1.0f)
#define T3_MOTOR_DIRECTION      (+1.0f)

/* 10 ms 控制周期（100 Hz） */
#define T3_CONTROL_PERIOD_MS    (10U)

/* 实测默认 PID，可由上位机覆盖 */
#define T3_PID_KP_DEFAULT       (2.0f)
#define T3_PID_KI_DEFAULT       (0.25f)
#define T3_PID_KD_DEFAULT       (0.0f)

/* 单周期 PWM 增量限幅，抑制超调 */
#define T3_PID_MAX_DELTA        (1500.0f)

/* 目标转速上限（实测满占空比约 990 rpm） */
#define T3_TARGET_RPM_MAX       (1000.0f)

/* 上位机串口：UART3；接收优先级低于控制中断 */
#define T3_HOST_UART            (UART_3)
#define T3_HOST_BAUD            (UART_BAUD_115200)
#define T3_HOST_RX_PRIORITY     (5U)

/* 帧：A5 CMD LEN payload CRC16_L CRC16_H 5A
 * CRC16 覆盖 CMD+LEN+payload（0xA001，初值 0xFFFF），负载小端 */
#define T3_FRAME_HEAD           (0xA5U)
#define T3_FRAME_TAIL           (0x5AU)

#define T3_CMD_START            (0x01U)     /* 启动 */
#define T3_CMD_STOP             (0x02U)     /* 停止 */
#define T3_CMD_SET_TARGET       (0x03U)     /* 目标转速，1 个 float */
#define T3_CMD_SET_PID          (0x04U)     /* PID，3 个 float */
#define T3_CMD_QUERY            (0x05U)     /* 查询状态 */

/* 滑块指令：A5 13 04 + float 小端 + 5A，共 8 字节，无 CRC */
#define T3_CMD_SET_TARGET_SLIDER (0x13U)

/* 每 5 个控制周期回传一帧（20 Hz） */
#define T3_UPLOAD_TICKS         (5U)

#endif /* _BOARD_CONFIG_H_ */
