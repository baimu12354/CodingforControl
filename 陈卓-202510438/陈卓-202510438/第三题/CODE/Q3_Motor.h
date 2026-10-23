#ifndef _Q3_MOTOR_H_
#define _Q3_MOTOR_H_

#define LEFT_MOTOR_PWM_PIN       (TIM2_CH3_PA2)             //左马达pwm信号接口
#define LEFT_MOTOR_DIR_PIN       (PA5)                      //左马达方向接口
#define LEFT_MOTOR_ECODER_PIN    (Encoder_TIM4)             //左马达编码器定时器接口
#define LEFT_MOTOR_ECODER_PIN_A  (Encoder_TIM4_PhaseA_PB6)  //左马达pwmA相接口
#define LEFT_MOTOR_ECODER_PIN_B  (Encoder_TIM4_PhaseB_PB7)  //左马达pwmB相接口

#define RIGHT_MOTOR_PWM_PIN      (TIM8_CH1_PC6)             //右马达pwm信号接口
#define RIGHT_MOTOR_DIR_PIN      (PC7)                      //右马达方向接口
#define RIGHT_MOTOR_ECODER_PIN   (Encoder_TIM3)             //右马达编码器定时器接口
#define RIGHT_MOTOR_ECODER_PIN_A (Encoder_TIM3_PhaseA_PA6)  //右马达pwmA相接口
#define RIGHT_MOTOR_ECODER_PIN_B (Encoder_TIM3_PhaseB_PA7)  //右马达pwmB相接口

#define Q3_PWM_FREQ_HZ           (4200)     /* PWM 频率, 沿用实验室驱动板惯用值;
                                              库内 PWM 分频限制约 8.4kHz 上限 */

#define Q3_CTRL_PERIOD_MS       (10)       /* 闭环固定周期: 100Hz 控制计算与 PWM 更新 */

#define Q3_ENC_COUNTS_PER_REV   (450)       /* 编码器单圈计数 = 线数 x 4 倍频(默认 13 线);
                                              !!按实际电机标定: 手转一圈看一圈计数!! */
#define Q3_ENC_SIGN             (1)        /* 编码器计数方向与电机正转的符号对应;
                                              若 START 后正反馈飞转, 改为 (-1) */

/* 增量 PID 出厂默认值(上位机字节包可在线修改, 无需重编译) */
#define Q3_PID_KP_DEF           (20.0f)
#define Q3_PID_KI_DEF           (0.0f)
#define Q3_PID_KD_DEF           (0.5f)

#define Q3_TARGET_MAX_ABS       (10000.0f) /* 目标转速限幅(rpm), 拦截无效数据 */

/* ---------------- 命令字节包协议(下行: 上位机 -> MCU) ----------------
 * 帧格式(自定义, 总长 5+LEN):
 *   [A5][5A][CMD][LEN][PAYLOAD(LEN 字节)][CHK]
 *   CHK = (CMD + LEN + PAYLOAD 各字节和) & 0xFF
 * 支持: 一帧分次到达、多帧连发、坏帧自动重新同步(帧内超时 100ms 复位)。
 * 所有多字节标量按小端序传输(与 STM32 内存序一致)。
 */
#define Q3_FRAME_HDR1           (0xA5)
#define Q3_FRAME_HDR2           (0x5A)
#define Q3_CMD_START            (0x01)     /* LEN=0            启动电机闭环      */
#define Q3_CMD_STOP             (0x02)     /* LEN=0            停机(PWM=0, PID复位) */
#define Q3_CMD_TARGET           (0x03)     /* LEN=4  float rpm 设置目标转速, 运行中/停止均可 */
#define Q3_CMD_P                (0x04)     /* LEN=4 kp,ki,kd  在线整定, 立即生效 */
#define Q3_CMD_I                (0x05)
#define Q3_CMD_D                (0x06)
#define Q3_CMD_QUERY            (0x07)     /* LEN=0            立即回传一帧遥测 */
#define Q3_CHK_BYPASS           (0x08)
#define Q3_RX_PAYLOAD_MAX       (16)

/* ---------------- 遥测帧(上行: MCU -> 上位机) ----------------
 * VOFA+ JustFloat: 7 个 float + 帧尾, 100Hz(每控制周期一帧, 主循环发送)
 *   CH1 目标转速 rpm | CH2 实际转速 rpm | CH3 Kp | CH4 Ki | CH5 Kd
 *   CH6 PWM 输出(-10000~10000) | CH7 运行状态(1=运行, 0=停止)
 * 上位机显示/绘制一律以本回传帧为准(当前生效值)。
 */
#define Q3_TELEM_CH_NUM         (7)
#define Q3_TELEM_BYTES          (Q3_TELEM_CH_NUM * 4 + 4)   /* 32 字节 */

/* ---------------- 对外状态(遥测/调试可见) ---------------- */
extern volatile float    Q3_target_rpm;    /* 当前生效目标转速(rpm) */
extern volatile float    Q3_actual_rpm;    /* 最近一次测得实际转速(rpm) */
extern volatile float    Q3_duty;          /* PID 累积输出(-10000~10000) */
extern volatile uint8_t  Q3_running;       /* 1=闭环运行 0=待命(上电默认) */

/* ---------------- API ---------------- */
void Q3_Motor_Init(void);                                  /* 上电初始化: PWM/方向/编码器/PID/FIFO */
void Q3_Motor_Tick(void);                                  /* 定时中断内调用: 测速+PID+PWM(每控制周期) */
void Q3_Motor_OnRxByte(uint8_t dat);                       /* 串口 RX 中断内调用: 字节入 FIFO */
void Q3_Rx_Poll(void);                                     /* 主循环内调用: 解包命令+上传遥测 */

#endif


