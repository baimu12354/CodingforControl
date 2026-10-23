#include "NEUQ_lib_include.h"

// ==================== 全局变量 ====================
volatile float motor_actual_speed = 0;   // 实际转速 rpm，控制中断里更新，主循环/协议层读
volatile uint32_t motor_tick_ms = 0;     // 系统节拍，每 CONTROL_PERIOD_MS 加一次

static int32_t encoder_last = 0;         // 上一次编码器计数（测速用）

// ==================== PID 部分（核心！）====================
// 说明：采用「增量式 PID」，输入目标转速和实际转速，输出 PWM 控制量。
//       Δu = Kp*(e-e0) + Ki*e + Kd*(e-2e0+e1)
//       u(k) = u(k-1) + Δu   （输出在上一拍基础上累加）
typedef struct {
    float kp, ki, kd;     // 三个参数，由串口 CMD_SET_PID 设置
    float last_error;     // 上一次误差 e(k-1)
    float prev_error;     // 上上次误差 e(k-2)，算微分用
    float last_output;    // 上一次输出 u(k-1)，累加用
} MotorPID_t;

static MotorPID_t motor_pid;    // PID 状态，只在控制中断里使用

// 增量式 PID 更新函数
//   target        目标转速（rpm）
//   actual        实际转速（rpm）
//   返回值        PID 输出，即给电机的 PWM 控制量（有符号）
static float Motor_PID_Update(MotorPID_t *pid, float target, float actual){
    float error = target - actual;   // ① 本次误差 = 目标 - 实际

    // ② 增量式核心：先算"输出该变化多少" Δu（而不是绝对输出）
    float delta_u = pid->kp * (error - pid->last_error)                    // 比例项：误差的变化量
                  + pid->ki *  error                                       // 积分项：本次误差（不再累加）
                  + pid->kd * (error - 2.0f*pid->last_error + pid->prev_error); // 微分项

    // ③ 本次输出 = 上次输出 + 变化量
    float output = pid->last_output + delta_u;

    // ④ 输出限幅（防 PWM 越界，也防输出无限累积）
    if(output >  PWM_MAX) output =  PWM_MAX;
    if(output < -PWM_MAX) output = -PWM_MAX;

    // ⑤ 历史前移一格，供下次计算
    pid->prev_error  = pid->last_error;
    pid->last_error  = error;
    pid->last_output = output;

    return output;
}

// 复位 PID 状态（停止时调用，清空历史，防止下次启动带着旧值）
static void Motor_PID_Reset(MotorPID_t *pid){
    pid->last_error  = 0;
    pid->prev_error  = 0;
    pid->last_output = 0;
}

// ==================== 电机输出（PWM + 方向）====================
// out 是有符号的 PWM 值，范围 [-10000, +10000]
// 正数 = 正转，负数 = 反转
static void Motor_SetOutput(float out){
    if(out >  PWM_MAX) out =  PWM_MAX;   // 限幅
    if(out < -PWM_MAX) out = -PWM_MAX;

    if(out >= 0){
        NEUQ_GPIO_write(MOTOR_DIR_PIN, MOTOR_DIR_FORWARD);   // 正转
    }else{
        NEUQ_GPIO_write(MOTOR_DIR_PIN, MOTOR_DIR_REVERSE);   // 反转
    }

    uint32_t duty = (out >= 0) ? (uint32_t)out : (uint32_t)(-out);   // 取绝对值作为占空比
    NEUQ_PWM_set_duty(MOTOR_PWM_PIN, duty);
}

// ==================== 初始化 ====================
void NEUQ_Motor_Init(void){
    // 1. PWM 调速引脚
    NEUQ_PWM_init(MOTOR_PWM_PIN, PWM_FREQ, 0);

    // 2. 方向脚
    NEUQ_GPIO_init(MOTOR_DIR_PIN, GPIO_Mode_Output_PushPull, MOTOR_DIR_FORWARD);

    // 3. 编码器
    NEUQ_Encoder_init(ENCODER_DEV, ENCODER_PHASE_A, ENCODER_PHASE_B);

    // 4. 状态清零 + 装载 PID 参数（Kp/Ki/Kd 由协议层 motor_kp/ki/kd 维护）
    motor_pid.kp = motor_kp;
    motor_pid.ki = motor_ki;
    motor_pid.kd = motor_kd;
    Motor_PID_Reset(&motor_pid);
    encoder_last = NEUQ_Encoder_Read(ENCODER_DEV);
    motor_actual_speed = 0;

    // 5. 先确保电机停止（上电后等待启动命令）
    Motor_SetOutput(0);

    // 6. 启动固定周期控制中断（TIM6，每 CONTROL_PERIOD_MS 一次）
    NEUQ_PIT_init_ms(PIT_CH1, CONTROL_PERIOD_MS);
}

// ==================== 控制中断（每 CONTROL_PERIOD_MS 执行一次）====================
void NEUQ_Motor_Control_Handler(void){
    // 1. 编码器测速（M 法）：本周期计数差 → 换算成 rpm
    int32_t now = NEUQ_Encoder_Read(ENCODER_DEV);
    int32_t delta = now - encoder_last;        // 这个周期内转了多少计数
    // 编码器计数是 16 位，会从 +32767 回绕到 -32768（或反过来），
    // 这里把回绕产生的巨大差值纠正回正常的小差值，否则会测出几百万转
    if(delta >  32767) delta -= 65536;
    if(delta < -32768) delta += 65536;
    encoder_last = now;

    // 转速换算：rpm = 计数差 * 60000 / (每圈计数 * 周期ms)
    // 每圈计数 = 线数 * 4（四倍频）
    motor_actual_speed = (float)delta * 60000.0f
                       / (ENCODER_PPR * 4.0f * CONTROL_PERIOD_MS)
                       * ENCODER_DIR_SIGN;

    motor_tick_ms += CONTROL_PERIOD_MS;        // 节拍累加

    // 2. 停止状态：同步最新 Kp/Ki/Kd，输出 0，并复位 PID
    if(motor_run == 0){
        motor_pid.kp = motor_kp;   // 参数只在停止时允许改，这里每周期把最新值搬进 PID
        motor_pid.ki = motor_ki;
        motor_pid.kd = motor_kd;
        Motor_PID_Reset(&motor_pid);
        Motor_SetOutput(0);
        return;
    }

    // 3. 运行状态：PID 闭环计算
    float pid_out = Motor_PID_Update(&motor_pid,
                                     (float)motor_target_speed,
                                     motor_actual_speed);

    // 4. 输出到电机
    Motor_SetOutput(pid_out);
}
