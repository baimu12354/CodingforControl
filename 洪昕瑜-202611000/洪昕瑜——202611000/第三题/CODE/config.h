#ifndef CONFIG_H
#define CONFIG_H

// ===== 电机 PWM / 方向 / 使能 =====
#define MOTOR_PWM_PORT   TIM3_CH1_PC6        // 电机 PWM 输出引脚 (PWM_1_IN)
#define MOTOR_PWM_FREQ   8000                // PWM 频率(换算后约 8.4kHz)
#define MOTOR_DIR        PB12                // 电机方向脚 (DIR_1_IN)
#define MOTOR_EN         PB13                // 电机使能脚 (EN_1)

// ===== 编码器 =====
#define ENCODER_DEV      Encoder_TIM4
#define ENCODER_A        Encoder_TIM4_PhaseA_PB6  // 编码器 A 相
#define ENCODER_B        Encoder_TIM4_PhaseB_PB7  // 编码器 B 相
#define ENCODER_LINES    440                 // 输出轴一圈脉冲数 = 11线 × 4倍频 × 10减速比 (编码器直测电机轴, 显示输出轴转速)

// ===== 测速 =====
#define FAST_WINDOW_MS   12                  // 快速测速窗口(误差大时用): 越小响应越快
#define FINE_WINDOW_MS   30                  // 精测窗口(误差小时用): 越大分辨率越高但滞后越大
#define ADAPT_ERR_FINE   15.0f               // |目标-实际| <此值 → 切精窗
#define ADAPT_ERR_FAST   40.0f               // |目标-实际| >此值 → 切快窗, 两者之间保持当前(迟滞防抖)
#define SPEED_PERIOD_MS  5                   // 测速/PID反馈更新周期(ms), 越小响应越快, 与上报解耦
#define SPEED_LPF_ALPHA  0.5f                // 测速低通滤波系数(0~1, 越小越平滑但滞后越大)
// 快/精两种窗口的计数 -> 转速 RPM 换算系数(ENCODER_LINES 是输出轴一圈脉冲数, 已含四倍频和减速比)
#define FAST_SCALE       (60000.0f / ((ENCODER_LINES) * (FAST_WINDOW_MS)))
#define FINE_SCALE       (60000.0f / ((ENCODER_LINES) * (FINE_WINDOW_MS)))

// ===== PID =====
#define MOTOR_MAX_RPM    1050.0f             // 输出轴最高转速, 速度前馈基准 FF = 目标/此值 × 满占空比
#define BANG_RPM         70.0f               // 误差 > 此值 → 满占空比正向加速(响应快); 越小越激进(满油门越久, 越快但超调风险高)
#define BANG_HYST        30.0f               // bang 退出滞回: 误差降到 BANG_RPM-HYST 以下才退出满油门, 防阈值抖动

// ===== 控制 =====
#define CTRL_PERIOD_MS   1                   // 控制周期 1ms (PIT 定时)
#define RAMP_RATE_UP     12.0f               // 加速斜坡(RPM/ms): 快, 响应时间约为目标/RAMP_RATE_UP
#define RAMP_RATE_DOWN   6.0f                // 减速斜坡(RPM/ms): 慢, 匹配物理减速能力, 防反向刹车过猛→反超调

// ===== 串口 =====
#define COMM_UART        UART_0              // 通信串口 (UART_0=PA9/PA10)
#define COMM_BAUD        UART_BAUD_115200

// ===== 电池电压采样 =====
#define VOLT_ADC_CH      ADC1_CH1_PA1       // 电池电压采样脚
#define VOLT_DIV_RATIO   4.0f               // 分压比 = (R上+R下)/R下
#define VOLT_VREF        3.3f               // ADC 参考电压(V), 一般为 3.3V
#define VOLT_SAMPLE_MS   100                // 电压采样周期(ms)

// ===== 协议帧 =====
#define FRAME_HEAD1      0xAA
#define FRAME_HEAD2      0x55
#define FRAME_DATA_MAX   12

// ===== 命令字 =====
#define CMD_START        0x01
#define CMD_STOP         0x02
#define CMD_SPEED        0x03
#define CMD_KP           0x05
#define CMD_KI           0x06
#define CMD_KD           0x07

// ===== 上报 =====
#define REPORT_PERIOD_MS 10

#endif
