#ifndef NEUQ_MOTOR_H_
#define NEUQ_MOTOR_H_

// 编码器参数（单位 rev/s，指输出轴）
#define ENCODER_PPR           11.0f   // 编码器每转脉冲数（单相线数）
#define ENCODER_MULTIPLIER     4.0f   // 正交四倍频
#define PIT1_PERIOD_MS         20.0f  // PIT1 周期
#define MIN_DUTY               0.0f   // 死区补偿（只在 |duty|>阈值 时叠加）
#define GEAR_RAT               10.0f  // 减速比：转子 / 输出轴

// PWM 满量程：按开环 3000 = 30% 推定 10000
#ifndef PWM_MAX
#define PWM_MAX                10000.0f
#endif

// 编码器方向校准：若正转时反馈为负，改成 -1.0f
#define ENCODER_L_DIR          (1.0f)
#define ENCODER_R_DIR          (1.0f)

extern int32_t curspeed_lv;
extern int32_t curspeed_rv;
extern volatile float actual_speed_lv;       // 左侧输出轴转速 (rev/s)
extern volatile float actual_speed_rv;       // 右侧输出轴转速 (rev/s)

extern float duty_lv;
extern float duty_rv;
extern float delta_duty_lv;
extern float delta_duty_rv;

extern volatile uint8_t pid_modify_rejected;

void Motor_PID_Init(void);
float Encoder_Counts_To_Speed(int32_t counts);   // counts/周期 -> rev/s
void Motor_Contral_LX(void);
void Motor_Contral_RX(void);

#endif