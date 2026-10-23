/*********************************************************************************************************************
* 文件名称          Task3_Motor
* 功能说明          第三题：串口字节包调参 + 电机速度闭环（实现），说明见 Task3_Motor.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "Task_Config.h"
#include "Task3_Motor.h"

#if (TASK_SELECT == 3)

#include "My_Vofa.h"
#include "Task3_Protocol.h"
#include "Task3_PID.h"

// NEUQ_Uart.c 里实现了"开启串口接收中断"的函数，但 NEUQ_Uart.h 里没有声明，这里补上声明（不修改库文件）
void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);
// NEUQ_Uart.c 里的串口句柄和单字节接收缓存，串口出错后重新开启接收时要用
extern UART_HandleTypeDef UART_0_handle, UART_1_handle, UART_2_handle, UART_3_handle;
extern uint8_t            UART_0_RX_dat, UART_1_RX_dat, UART_2_RX_dat, UART_3_RX_dat;

#define T3_UART                 ((UART_DEV_ENUM)T3_UART_INDEX)
#define T3_PWM_FULL_SCALE       (10000.0f)      // 库的 PWM 满量程（NEUQ_PWM.h 中的 PWM_MAX）

// 转速换算：输出轴转一圈编码器计数 CPR = 线数 × 4 倍频 × 减速比（11 × 4 × 10 = 440，1 个计数约 27.3 rpm）
// 每个控制周期（5 ms）读到 N 个计数，则 转速(rpm) = N / CPR / 0.005 s × 60 = N × 60 × 1000 / (CPR × 5)
#define T3_COUNTS_PER_REV       (T3_ENCODER_LINES * T3_ENCODER_MULTIPLE * T3_GEAR_RATIO)
#define T3_COUNTS_TO_RPM        (60.0f * 1000.0f / (T3_COUNTS_PER_REV * (float)T3_CONTROL_PERIOD_MS))

enum { STATE_STOP = 0, STATE_RUN = 1, STATE_FAULT = 2 };                            // 控制器状态
enum { RESULT_OK = 0, RESULT_REJECT_RUNNING = 1, RESULT_BAD_VALUE = 2, RESULT_BAD_FRAME = 3 };  // 指令执行结果

//==================================================== 控制器状态 ====================================================
static T3_Pid_Struct    s_pid;
static float            s_target      = 0.0f;   // 上位机设定的目标转速 rpm
static float            s_speed       = 0.0f;   // 实际转速 rpm（滤波后）
static float            s_output      = 0.0f;   // 本周期 PWM 输出（带符号，正数正转）
static uint8_t          s_state       = STATE_STOP;   // 上电为停止，等待 START 指令
static uint8_t          s_last_result = RESULT_OK;
static uint32_t         s_frame_count = 0;      // 收到的完整的帧数
static uint16_t         s_fault_ms    = 0;      // 接反保护计时
static float            s_fault_abs_ema = 0.0f; // |转速| 的慢速基准（约 100 ms 一阶），区分"飞车"与"减速"
static uint8_t          s_telemetry_timer = 0;  // 5 ms 中断里数到 2 就回传一次

// 串口接收：中断把字节放进 FIFO，主循环取出来解析
static uint8_t          s_rx_buffer[256];
static NEUQ_FIFO_t      s_rx_fifo;
static T3_Parser_Struct s_parser;

//==================================================== 硬件接口 ====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     输出到电机驱动：正数正转、负数反转，绝对值是占空比（0 ~ 10000 对应 0 ~ 100%）
// 备注信息     DRV8701E 是 PH/EN 接口：方向脚给电平，PWM 脚给占空比（具体引脚见 Task_Config.h）。
//              换向先停 PWM 一个控制周期，等定时器装载零占空比后再切换方向。
//-------------------------------------------------------------------------------------------------------------------
static void t3_motor_output(float out)
{
    static uint8_t reverse = 0;       // 与上电设置的正方向电平一致
    static uint8_t waiting = 0;       // 上一拍是否已经先把 PWM 置零
    uint8_t next_reverse = (out < 0.0f) ? 1U : 0U;

    if (out == 0.0f) {
        NEUQ_PWM_set_duty(T3_MOTOR_PWM, 0);
        waiting = 0;
        return;                       // 停止时保留方向，EN=0 为驱动器制动状态
    }
    if (next_reverse != reverse) {
        if (!waiting) {
            NEUQ_PWM_set_duty(T3_MOTOR_PWM, 0);
            waiting = 1;
            return;
        }
        NEUQ_GPIO_write(T3_MOTOR_DIR, next_reverse ? T3_MOTOR_REVERSE_LEVEL : T3_MOTOR_FORWARD_LEVEL);
        reverse = next_reverse;
    }
    waiting = 0;
    NEUQ_PWM_set_duty(T3_MOTOR_PWM, (uint32_t)((out < 0.0f ? -out : out) + 0.5f));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取这个控制周期（5 ms）内编码器转过了多少个计数，正转为正、反转为负
// 备注信息     库的用法：NEUQ_Encoder_Read() 读出从上次清零到现在的计数，NEUQ_Encoder_Clear() 清零重新计数。
//              每个控制周期"读一次、清一次"，读到的就是这 5 ms 的计数
//-------------------------------------------------------------------------------------------------------------------
static int32_t t3_encoder_count(void)
{
    int32_t count = NEUQ_Encoder_Read(T3_ENCODER_TIM);
    NEUQ_Encoder_Clear(T3_ENCODER_TIM);
#if T3_ENCODER_DIR_INVERT
    count = -count;                         // 编码器方向装反了：取反，保证"正 PWM 时转速为正"
#endif
    return count;
}

// 目标、输出都要求反向，但编码器持续报告原方向：可能是方向线或反馈异常。
// 正常换向会先减速，明显减速时不累计；错误方向稳定在高速时也必须能停机。
static uint8_t t3_direction_fault(float target, float speed, float output)
{
    float abs_speed = (speed < 0.0f) ? -speed : speed;
    s_fault_abs_ema += 0.05f * (abs_speed - s_fault_abs_ema);
    if (target * speed < 0.0f && output * speed < 0.0f &&
        (output > 0.9f * T3_PWM_LIMIT || output < -0.9f * T3_PWM_LIMIT) &&
        abs_speed > T3_FAULT_SPEED_RPM &&
        abs_speed >= s_fault_abs_ema - T3_FAULT_DECEL_RPM) {
        s_fault_ms += T3_CONTROL_PERIOD_MS;
        if (s_fault_ms >= T3_FAULT_TIME_MS) return 1;
    }
    else {
        s_fault_ms = 0;
    }
    return 0;
}

//==================================================== 控制 ====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     5 ms 控制中断：测速 → PID → PWM，并定时回传
// 备注信息     1. 测速：这 5 ms 的计数换算成 rpm，再做一阶低通（0.5 × 新值 + 0.5 × 旧值）减小计数量化带来的抖动
//              2. 运行中才做 PID；停止 / 故障时输出 0
//              3. 接反保护：输出已接近满幅，目标和输出都与实际转速反向，且没有明显减速，可能是方向线或反馈异常
//                 持续 T3_FAULT_TIME_MS 就自动停机，状态变为故障
//-------------------------------------------------------------------------------------------------------------------
void Task3_PIT_Handler(void)
{
    float rpm = (float)t3_encoder_count() * T3_COUNTS_TO_RPM;
    s_speed += T3_SPEED_FILTER_ALPHA * (rpm - s_speed);

    if (s_state == STATE_RUN) {
        s_output = T3_Pid_update(&s_pid, s_target, s_speed);

        if (t3_direction_fault(s_target, s_speed, s_output)) {
            s_state = STATE_FAULT;
            s_output = 0.0f;
        }
    }
    else {
        s_output = 0.0f;
        s_fault_ms = 0;
        s_fault_abs_ema = (s_speed < 0.0f) ? -s_speed : s_speed;
    }
    t3_motor_output(s_output);

    // 回传：这些数就是控制器此刻正在使用的值，上位机看到的就是实际生效的参数
    s_telemetry_timer++;
    if (s_telemetry_timer >= T3_TELEMETRY_DIVIDER) {
        float channel[10];
        s_telemetry_timer = 0;
        channel[0] = (s_state == STATE_RUN) ? s_target : 0.0f;         // I0 控制器正在跟踪的目标（停止时为 0）
        channel[1] = s_speed;                                           // I1 实际转速 rpm
        channel[2] = s_target;                                          // I2 上位机请求的目标转速 rpm
        channel[3] = s_pid.kp;                                          // I3 ~ I5 当前生效的 PID 参数
        channel[4] = s_pid.ki;
        channel[5] = s_pid.kd;
        channel[6] = (float)s_state;                                    // I6 状态 0 停止 1 运行 2 故障
        channel[7] = s_output * 100.0f / T3_PWM_FULL_SCALE;             // I7 PWM 输出 %
        channel[8] = (float)s_frame_count;                              // I8 收到的有效帧数
        channel[9] = (float)s_last_result;                              // I9 最近一条指令的结果
        My_Vofa_push(channel, 10);
    }
}

//==================================================== 指令 ====================================================
// 检查 float 是正常数字（排除 NaN 和无穷大）；NaN 和任何数比较都为假
static uint8_t t3_is_number(float x)
{
    return (x > -3.0e38f && x < 3.0e38f) ? 1U : 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一条完整的指令
// 返回参数     执行结果（RESULT_xxx），被拒绝的指令不会改变任何参数
// 备注信息     START       停止 / 故障 → 运行，同时清空 PID 的积分和历史
//              STOP        立即停止，输出 0
//              SET_TARGET  任何时候都能改；超过 ±T3_TARGET_LIMIT_RPM 会被限幅
//              SET_PID     任务书要求只能在停止状态修改，运行中收到会拒绝
//-------------------------------------------------------------------------------------------------------------------
static uint8_t t3_execute(const T3_Frame_Struct *frame)
{
    if (frame->cmd == T3_CMD_START && frame->len == 0U) {
        if (s_state != STATE_RUN) {
            T3_Pid_reset(&s_pid);
            s_fault_ms = 0;
            s_state    = STATE_RUN;
        }
        return RESULT_OK;
    }
    if (frame->cmd == T3_CMD_STOP && frame->len == 0U) {
        s_state = STATE_STOP;
        s_output = 0.0f;
        t3_motor_output(0.0f);          // 收到完整 STOP 帧后立即将 PWM 置零
        T3_Pid_reset(&s_pid);
        return RESULT_OK;
    }
    if (frame->cmd == T3_CMD_SET_TARGET && frame->len == 2U) {
        float target = (float)T3_Frame_get_target(&frame->data[0]);
        if (!t3_is_number(target)) {
            return RESULT_BAD_VALUE;
        }
        if (target > T3_TARGET_LIMIT_RPM)  target = T3_TARGET_LIMIT_RPM;
        if (target < -T3_TARGET_LIMIT_RPM) target = -T3_TARGET_LIMIT_RPM;
        s_target = target;
        return RESULT_OK;
    }
    if (frame->cmd == T3_CMD_SET_PID && frame->len == 6U) {
        float kp = (float)T3_Frame_get_u16(&frame->data[0]) / 100.0f;
        float ki = (float)T3_Frame_get_u16(&frame->data[2]) / 100.0f;
        float kd = (float)T3_Frame_get_u16(&frame->data[4]) / 100.0f;
        if (s_state != STATE_STOP) {
            return RESULT_REJECT_RUNNING;
        }
        if (!t3_is_number(kp) || !t3_is_number(ki) || !t3_is_number(kd) ||
            kp < 0.0f || ki < 0.0f || kd < 0.0f ||
            kp > T3_PID_PARAM_MAX || ki > T3_PID_PARAM_MAX || kd > T3_PID_PARAM_MAX) {
            return RESULT_BAD_VALUE;
        }
        T3_Pid_init(&s_pid, kp, ki, kd, T3_PWM_LIMIT);
        return RESULT_OK;
    }
    return RESULT_BAD_FRAME;                // 未知命令，或者长度与命令不符
}

//==================================================== 串口 ====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口接收中断：只把字节放进 FIFO，解析交给主循环（中断越短越好）
//-------------------------------------------------------------------------------------------------------------------
void Task3_UART_RX_Handler(uint8_t byte)
{
    NEUQ_FIFO_write_byte(&s_rx_fifo, byte);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HAL 串口出错回调（这里的定义会替换 HAL 库里的空函数）
// 备注信息     上一个字节还没被取走下一个字节就到了（溢出错误 ORE）时，HAL 会停止中断接收，
//              这里重新开启，保证长时间连续收发时接收不会"突然停了"
//-------------------------------------------------------------------------------------------------------------------
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)      HAL_UART_Receive_IT(&UART_0_handle, &UART_0_RX_dat, 1);
    else if (huart->Instance == USART2) HAL_UART_Receive_IT(&UART_1_handle, &UART_1_RX_dat, 1);
    else if (huart->Instance == USART3) HAL_UART_Receive_IT(&UART_2_handle, &UART_2_RX_dat, 1);
    else if (huart->Instance == UART4)  HAL_UART_Receive_IT(&UART_3_handle, &UART_3_RX_dat, 1);
}

//==================================================== 主函数 ====================================================
void Task3_Main(void)
{
    T3_Frame_Struct frame;
    uint8_t         byte;

    NEUQ_Debug_init();                                          // UART_0 调试串口
    NEUQ_Uart_init(T3_UART, T3_UART_BAUD);                      // 上位机串口 UART_3：PC10 TX / PC11 RX
    NEUQ_Delay_init();
    printf("Task3 motor start, waiting for START frame\r\n");   // 这之后串口上只有二进制数据

    // 电机：先设置方向电平，再以零占空比启动定时器（引脚见 Task_Config.h 的 T3_MOTOR_DIR / T3_MOTOR_PWM）。
    NEUQ_GPIO_init(T3_MOTOR_DIR, GPIO_Mode_Output_PushPull, T3_MOTOR_FORWARD_LEVEL);
    NEUQ_PWM_init(T3_MOTOR_PWM, T3_MOTOR_PWM_FREQ_HZ, 0);

    // 编码器（库在初始化时已经把计数清零）
    NEUQ_Encoder_init(T3_ENCODER_TIM, T3_ENCODER_PHASE_A, T3_ENCODER_PHASE_B);

    T3_Pid_init(&s_pid, T3_DEFAULT_KP, T3_DEFAULT_KI, T3_DEFAULT_KD, T3_PWM_LIMIT);
    T3_Parser_init(&s_parser);
    NEUQ_FIFO_init(&s_rx_fifo, s_rx_buffer, sizeof(s_rx_buffer));
    My_Vofa_init(T3_UART);

    NEUQ_Uart_Enable_RX_Interrupt(T3_UART, T3_UART_RX_PRIORITY);    // 串口接收中断，优先级 3
    NEUQ_PIT_init_ms(PIT_CH1, T3_CONTROL_PERIOD_MS);                // 控制中断，库默认优先级 0（最高）

    while (1) {
        // 1. 解析收到的字节：一帧分几次到达、几帧连在一起，都由解析器状态机处理
        while (NEUQ_FIFO_read_byte(&s_rx_fifo, &byte) == NEUQ_FIFO_OK) {
            if (T3_Parser_feed(&s_parser, byte, &frame)) {
                // 执行指令时短暂关中断：保证控制中断不会用到"只改了一半"的参数（比如 kp 改了 ki 还没改）
                __disable_irq();
                s_frame_count++;
                s_last_result = t3_execute(&frame);
                __enable_irq();
            }
        }
        // 2. 把回传数据发给上位机
        My_Vofa_poll();
    }
}

#endif
