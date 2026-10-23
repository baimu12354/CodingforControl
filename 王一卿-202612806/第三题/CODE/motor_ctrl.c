/*
 * 文件名称    motor_ctrl
 * 功能说明    串口字节包调参 + 编码器测速 + 电机速度闭环(PID)控制
 * 适用平台    STM32F407VE (NEUQ 开源库)
 *
 * 通信协议(下行，上位机 -> 单片机)，帧格式自定义：
 * [0xAA][0x55][CMD][LEN][DATA...][SUM]
 * SUM = (CMD + LEN + DATA 各字节之和) 的低 8 位 (当前固件已忽略校验和, 便于 VOFA+ 滑块等无法动态算 SUM 的上位机)
 * CMD_START  0x01 启动      LEN=0
 * CMD_STOP   0x02 停止      LEN=0
 * CMD_SPEED  0x03 目标转速  LEN=4  DATA=float(RPM, 小端)
 * CMD_PID    0x04 PID 参数  LEN=12 DATA=float Kp,Ki,Kd (仅停止时, 上位机脚本用)
 * CMD_KP     0x05 Kp       LEN=4  DATA=float (VOFA+ 滑块, 运行中也可实时调)
 * CMD_KI     0x06 Ki       LEN=4  DATA=float (同上)
 * CMD_KD     0x07 Kd       LEN=4  DATA=float (同上)
 *
 * 通信协议(上行，单片机 -> 上位机)，VOFA+ FireWater 文本帧：
 * "ch0,ch1,...,ch6\n"  (英文逗号分隔, 换行结尾)
 * 通道顺序: 目标转速, 实际转速, Kp, Ki, Kd, 占空比(%), 供电电压(V)
 */
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "motor_ctrl.h"
#include <stdio.h>
#include <string.h>

/* NEUQ_Uart.h 未声明该函数(库缺口), 这里补前置声明, 避免隐式声明告警 */
extern void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);


/*
 * 硬件配置
 */
#define MOTOR_PWM_PORT   TIM3_CH1_PC6        // 电机 PWM 输出引脚 (PWM_1_IN)
#define MOTOR_PWM_FREQ   8000                // PWM 频率(库换算后约 8.4kHz)
#define MOTOR_DIR        PB12                // 电机方向脚 (DIR_1_IN)
#define MOTOR_EN         PB13                // 电机使能脚 (EN_1, 必须保持高电平)

#define ENCODER_DEV      Encoder_TIM4
#define ENCODER_A        Encoder_TIM4_PhaseA_PB6  // 编码器 A 相 (PB6)



#define ENCODER_B        Encoder_TIM4_PhaseB_PB7  // 编码器 B 相 (PB7)
#define ENCODER_LINES    44                  // 电机轴一圈脉冲数 = 11线 × 4倍频 (编码器直测电机轴, 显示电机轴转速, 比输出轴快10倍)
#define ENCODER_USE_PULLUP 0                 // 此板子上内部上拉会导致串断，保持0，改用外部上拉电阻

#define COMM_UART        UART_0              // 通信串口 (UART_0=PA9/PA10)
#define COMM_BAUD        UART_BAUD_115200

#define VOLT_ADC_CH      ADC1_CH1_PA1       // 电池电压采样脚(需外部分压电阻, 严禁直连电池!)
#define VOLT_DIV_RATIO   4.0f               // 分压比 = (R上+R下)/R下, 例: 上30k 下10k = 4
#define VOLT_VREF        3.3f               // ADC 参考电压(V), 一般为 3.3V
#define VOLT_SAMPLE_MS   100                // 电压采样周期(ms)

/*
 * 控制参数
 */
#define CTRL_PERIOD_MS   1                   // 控制周期 1ms (PIT 定时)
#define FAST_WINDOW_MS   12                  // 快速测速窗口(误差大时用): 越小响应越快(12ms=113.6RPM/计数, 瞬态够用)
#define FINE_WINDOW_MS   30                  // 精测窗口(误差小时用): 越大分辨率越高但滞后越大; 30ms=45.5RPM/计数, 贴合快且误差<20
#define ADAPT_ERR_FINE   150.0f              // |目标-实际| 小于此值 → 切精窗(50ms)
#define ADAPT_ERR_FAST   400.0f              // |目标-实际| 大于此值 → 切快窗(20ms), 两者之间保持当前(迟滞防抖)
#define SPEED_PERIOD_MS  5                   // 测速/PID反馈更新周期(ms), 越小响应越快, 与上报解耦
#define REPORT_PERIOD_MS 10                  // 上报周期 10ms
#define SPEED_LPF_ALPHA  0.5f                // 测速低通滤波系数(0~1, 越小越平滑但滞后越大)
// 快/精两种窗口的计数 -> 转速 RPM 换算系数(ENCODER_LINES 是电机轴一圈脉冲数, 已含四倍频)
#define FAST_SCALE       (60000.0f / ((ENCODER_LINES) * (FAST_WINDOW_MS)))
#define FINE_SCALE       (60000.0f / ((ENCODER_LINES) * (FINE_WINDOW_MS)))
#define MOTOR_MAX_RPM    10500.0f            // 电机轴最高转速(反电动势极限), 速度前馈基准 FF=目标/此值×满占空比
#define BANG_RPM         700.0f              // 误差>此值→满占空比正向加速(响应快); 越小越激进(满油门越久, 越快但超调风险高)
#define BANG_HYST        300.0f              // bang退出滞回: 误差降到BANG_RPM-HYST以下才退出满油门, 防阈值抖动
#define RAMP_RATE_UP     120.0f              // 加速斜坡(RPM/ms): 快, 响应时间≈目标/RAMP_RATE_UP
#define RAMP_RATE_DOWN   60.0f               // 减速斜坡(RPM/ms): 慢, 匹配物理减速能力, 防反向刹车过猛→反超调

/*
 * 协议常量
 */
#define FRAME_HEAD1      0xAA
#define FRAME_HEAD2      0x55
#define FRAME_DATA_MAX   12

#define CMD_START        0x01
#define CMD_STOP         0x02
#define CMD_SPEED        0x03
#define CMD_PID          0x04
#define CMD_KP           0x05
#define CMD_KI           0x06
#define CMD_KD           0x07

/*
 * 内部变量
 */
typedef struct { float Kp, Ki, Kd, integral, prev_error; } PID_t;
static PID_t pid;

static volatile uint8_t running = 0;
static volatile float target_rpm = 0.0f;
static volatile float target_ref = 0.0f;    // 斜坡整形后的目标(PID实际追踪它, 非PID手段让曲线平滑贴合)
static volatile float actual_rpm = 0.0f;
static volatile int32_t last_enc = 0;
static volatile int32_t pwm_duty = 0;        // 当前 PWM 占空比(0~PWM_MAX=0~100%), 上报调试用
static volatile float supply_voltage = 0.0f; // 供电电压(V), ADC 采样, 上报观察
static int32_t enc_ring[FINE_WINDOW_MS];     // 环形缓冲: 最近 FINE_WINDOW_MS 个 1ms 计数差(最长50ms)
static uint8_t enc_ri = 0;                   // 环形缓冲写指针
static int32_t enc_sum_fine = 0;             // 精窗(50ms)计数和
static int32_t enc_sum_fast = 0;             // 快窗(20ms)计数和
static uint8_t speed_tick = 0;               // 转速更新节拍计数
static uint8_t use_fast = 1;                 // 当前测速窗口: 1=快窗(20ms) 0=精窗(50ms)

static uint8_t rx_fifo_buf[256];             // 串口接收 FIFO(中断写，主循环读)
static NEUQ_FIFO_t rx_fifo;

typedef enum { S_HEAD1, S_HEAD2, S_CMD, S_LEN, S_DATA, S_SUM } RxState_t;
static RxState_t rx_state = S_HEAD1;
static uint8_t rx_cmd, rx_len, rx_sum, rx_cnt;
static uint8_t rx_buf[FRAME_DATA_MAX];
static uint32_t last_report = 0;
static uint32_t last_volt = 0;

/*
 * 底层辅助函数
 */
static float Bytes_To_Float(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);          // STM32 为小端，直接拷贝
    return f;
}

static void PID_Reset(void)
{
    pid.integral = 0;
    pid.prev_error = 0;
}

static float PID_Compute(PID_t *p, float target, float actual)
{
    float err = target - actual;
    static uint8_t in_bang = 0;

    // bang-bang 只正向满油门加速(响应快): 减速方向不反向满油门(否则冲过头震荡), 交给PID负误差+微分刹车
    if (!in_bang && err > BANG_RPM) { in_bang = 1; }
    else if (in_bang && err < (BANG_RPM - BANG_HYST)) { in_bang = 0; p->integral = 0; p->prev_error = err; }
    if (in_bang) return PWM_MAX;

    // 速度前馈: 目标越接近最高转速, 反电动势需要的占空比越接近满值,
    // 直接按目标给基准占空比, 积分只补小误差(否则积分要从 0 攒满 ~97% 占空比, 响应极慢)
    float ff = target / MOTOR_MAX_RPM * PWM_MAX;

    float out = ff + p->Kp * err + p->Ki * p->integral + p->Kd * (err - p->prev_error);
    p->prev_error = err;

    // 条件积分抗饱和: 输出已被限幅且误差还在往限幅方向推时, 停止累加(防前馈+积分顶满导致超调)
    int sat = (out >= PWM_MAX && err > 0.0f) || (out <= -PWM_MAX && err < 0.0f);
    if (!sat) p->integral += err;

    // 积分限幅: 积分项最多贡献占满占空比 PWM_MAX
    float imax = PWM_MAX / (p->Ki > 0.001f ? p->Ki : 0.001f);
    if (p->integral > imax) p->integral = imax;
    if (p->integral < -imax) p->integral = -imax;

    return out;
}

static void Motor_SetPwm(float out)
{
    int32_t duty = (int32_t)out;
    if(duty > PWM_MAX) duty = PWM_MAX;
    if(duty < -PWM_MAX) duty = -PWM_MAX;

    if (duty > 0) {                    // 正转
        NEUQ_GPIO_write(MOTOR_DIR, GPIO_High);
    } else if (duty < 0) {             // 反转
        NEUQ_GPIO_write(MOTOR_DIR, GPIO_Low);
        duty = -duty;
    }
    // duty==0 时仅占空比清零(方向不变)，电机停止
    pwm_duty = duty;                                  // 记录当前占空比, 供上报观察
    NEUQ_PWM_set_duty(MOTOR_PWM_PORT, (uint32_t)duty);
}

// 编码器输入上拉(开集/霍尔编码器必须上拉才能读到电平)
#if ENCODER_USE_PULLUP
static void Encoder_EnablePullup(void)
{
    GPIO_InitTypeDef gpio = {};
    gpio.Pin     = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode    = GPIO_MODE_AF_PP;
    gpio.Pull    = GPIO_PULLUP;
    gpio.Speed   = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);
}
#endif

/*
 * 帧解析
 */
static void Proto_Dispatch(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    switch (cmd)
    {
        case CMD_START:                 // 启动
            PID_Reset();
            running = 1;
            break;
        case CMD_STOP:                  // 停止
            running = 0;
            Motor_SetPwm(0);
            break;
        case CMD_SPEED:                 // 目标转速(运行中可改)
            if (len == 4)
            {
                target_rpm = Bytes_To_Float(data);
                // 滑块只发速度不发 START, 这里按速度自动启停: 非零→启动, 拖到0→停止
                if (target_rpm != 0.0f)
                {
                    if (!running) { PID_Reset(); running = 1; }   // 首次给非零速度时启动
                }
                else
                {
                    running = 0; Motor_SetPwm(0); PID_Reset();
                }
            }
            break;
        case CMD_PID:                   // PID 三参数(仅停止时, 上位机脚本用)
            if (len == 12 && !running)
            {
                pid.Kp = Bytes_To_Float(data);
                pid.Ki = Bytes_To_Float(data + 4);
                pid.Kd = Bytes_To_Float(data + 8);
                PID_Reset();
            }
            break;
        case CMD_KP: case CMD_KI: case CMD_KD:   // 单参数(VOFA+ 滑块, 运行中也可调)
            if (len == 4) {
                float *p = (cmd == CMD_KP) ? &pid.Kp : (cmd == CMD_KI) ? &pid.Ki : &pid.Kd;
                *p = Bytes_To_Float(data);
                if (!running) PID_Reset();
            }
            break;
        default:
            break;
    }
}

static void Proto_Feed(uint8_t b)
{
    switch (rx_state)
    {
        case S_HEAD1:
            if (b == FRAME_HEAD1) rx_state = S_HEAD2;
            break;
        case S_HEAD2:
            if (b == FRAME_HEAD2) rx_state = S_CMD;
            else if (b != FRAME_HEAD1) rx_state = S_HEAD1;
            break;
        case S_CMD:
            rx_cmd = b; rx_sum = b; rx_state = S_LEN;
            break;
        case S_LEN:
            rx_len = b; rx_sum += b;
            if (rx_len > FRAME_DATA_MAX) { rx_state = S_HEAD1; break; }
            rx_cnt = 0;
            rx_state = (rx_len == 0) ? S_SUM : S_DATA;
            break;
        case S_DATA:
            rx_buf[rx_cnt++] = b; rx_sum += b;
            if (rx_cnt >= rx_len) rx_state = S_SUM;
            break;
        case S_SUM:
            // 忽略校验和: VOFA+ 滑块的 %% 只能插入浮点小端字节, 无法动态计算 SUM, 故直接派发
            Proto_Dispatch(rx_cmd, rx_buf, rx_len);
            rx_state = S_HEAD1;
            break;
    }
}

/*
 * 周期上报 (VOFA+ FireWater 文本帧: "ch0,ch1,...,ch6\n")
 */
static void Report_Send(void)
{
    char buf[96];
    int len = snprintf(buf, sizeof(buf),
                       "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                       target_rpm, actual_rpm, pid.Kp, pid.Ki, pid.Kd,
                       pwm_duty / 100.0f,   // 占空比 0~100% (10000=满)
                       supply_voltage);
    NEUQ_Uart_write_data(COMM_UART, (uint8_t *)buf, len);
}

/*
  对外接口
 */
void MotorCtrl_Init(void)
{
    // 方向脚 + 使能脚
    NEUQ_GPIO_init(MOTOR_DIR, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_GPIO_init(MOTOR_EN, GPIO_Mode_Output_PushPull, GPIO_High);  // 使能必须常高
    // PWM(初始占空比 0)
    NEUQ_PWM_init(MOTOR_PWM_PORT, MOTOR_PWM_FREQ, 0);
    // 编码器
    NEUQ_Encoder_init(ENCODER_DEV, ENCODER_A, ENCODER_B);
    // 编码器输入数字滤波: 滤掉 A/B 线上的短毛刺
    TIM4->CR1   &= ~0x0001u;                            // 先停计数器(滤波配置需在禁用时改)
    TIM4->CR1    = (TIM4->CR1   & ~0x0300u) | 0x0200u;  // CKD=10 → 4分频(f_DTS=21MHz)
    TIM4->CCMR1  = (TIM4->CCMR1 & ~0xFCFCu) | 0xF0F0u;  // IC1F=IC2F=15 → 约714ns毛刺滤波
    TIM4->CR1   |= 0x0001u;                             // 重新启动计数器
#if ENCODER_USE_PULLUP
    Encoder_EnablePullup();  // 霍尔/开集编码器需要上拉
#endif
    // 串口 + 接收中断(优先级低于控制中断)
    NEUQ_Uart_init(COMM_UART, COMM_BAUD);
    NEUQ_Uart_Enable_RX_Interrupt(COMM_UART, 5);
    // 供电电压 ADC 采样
    adc_init(VOLT_ADC_CH, ADC_12BIT);
    // 接收 FIFO
    NEUQ_FIFO_init(&rx_fifo, rx_fifo_buf, sizeof(rx_fifo_buf));
    // PID 初始参数(上电后可用 CMD_PID 修改)
    pid.Kp = 1.0f; pid.Ki = 0.002f; pid.Kd = 0.002f;  // 电机轴量程(约10500RPM)下的初始增益
    PID_Reset();

    // 状态
    running = 0; target_rpm = 0; actual_rpm = 0;
    last_enc = NEUQ_Encoder_Read(ENCODER_DEV);

    // 控制周期中断(1ms)
    NEUQ_PIT_init_ms(PIT_CH1, CTRL_PERIOD_MS);
}

void MotorCtrl_PeriodCallback(void)
{
    // 1) 编码器计数差(含 16 位计数器回绕处理) + 快/精双滑动窗口累加
    int32_t cnt = NEUQ_Encoder_Read(ENCODER_DEV);
    int32_t delta = cnt - last_enc;
    last_enc = cnt;
    if (delta > 32768)       delta -= 65536;   // 正方向回绕
    else if (delta < -32768) delta += 65536;   // 负方向回绕

    enc_sum_fine += delta - enc_ring[enc_ri];  // 精窗(50ms): 加新减最旧
    // 快窗(20ms): 加新减"20ms前那一项"(enc_ri-FAST 那格在50ms环内, 尚未被本轮覆盖)
    enc_sum_fast += delta - enc_ring[(enc_ri - FAST_WINDOW_MS + FINE_WINDOW_MS) % FINE_WINDOW_MS];
    enc_ring[enc_ri] = delta;
    if (++enc_ri >= FINE_WINDOW_MS) enc_ri = 0;

    // 2) 每 REPORT_PERIOD_MS 更新一次转速 + 中值滤波 + 低通滤波
    if (++speed_tick >= SPEED_PERIOD_MS)
    {
        speed_tick = 0;

        // 自适应窗口: 误差大用快窗, 误差小用精窗; 迟滞防抖
        float e = target_rpm - actual_rpm; if (e < 0) e = -e;
        if (use_fast) { if (e < ADAPT_ERR_FINE) use_fast = 0; }
        else          { if (e > ADAPT_ERR_FAST) use_fast = 1; }
        float rpm = use_fast ? (float)enc_sum_fast * FAST_SCALE : (float)enc_sum_fine * FINE_SCALE;

        // 中值滤波: 最近3个原始测速样本取中值, 剔除偶发毛刺/尖峰(抗干扰)
        static float samp[3] = {0, 0, 0};
        static uint8_t si = 0;
        samp[si] = rpm;
        si = (si + 1) % 3;
        float a = samp[0], b = samp[1], c = samp[2];
        if (a > b) { float t = a; a = b; b = t; }
        if (b > c) { float t = b; b = c; c = t; }
        if (a > b) { float t = a; a = b; b = t; }
        float med = b;

        actual_rpm += SPEED_LPF_ALPHA * (med - actual_rpm);
    }

    // 2.5) 目标斜坡整形(非PID): target_ref 以 RAMP_RATE 逼近 target_rpm, 让响应曲线平滑贴合、少超调
    if (running)
    {
        if (target_ref < target_rpm) {
            target_ref += RAMP_RATE_UP;              // 加速: 快斜坡
            if (target_ref > target_rpm) target_ref = target_rpm;
        } else if (target_ref > target_rpm) {
            target_ref -= RAMP_RATE_DOWN;            // 减速: 慢斜坡, 防反向刹车过猛
            if (target_ref < target_rpm) target_ref = target_rpm;
        }
    } else {
        target_ref = actual_rpm;                     // 停止时跟随实际转速, 启动时从当前转速平滑爬
    }

    // 3) 固定周期 PID 计算 + PWM 更新
    if (running)
    {
        float out = PID_Compute(&pid, target_ref, actual_rpm);
        Motor_SetPwm(out);
    } else {
        Motor_SetPwm(0);
        PID_Reset();
    }
}

void MotorCtrl_RxByte(uint8_t dat)
{
    NEUQ_FIFO_write_byte(&rx_fifo, dat);
}

void MotorCtrl_Loop(void)
{
    // 1) 解析接收字节流(支持分帧到达 / 连续多帧)
    uint8_t b;
    while (NEUQ_FIFO_read_byte(&rx_fifo, &b) == NEUQ_FIFO_OK)
    {
        Proto_Feed(b);
    }

    uint32_t now = HAL_GetTick();

    // 2) 供电电压采样(100ms 一次, ADC 均值滤波; 不在 1ms 中断里做, 避免阻塞控制)
    if (now - last_volt >= VOLT_SAMPLE_MS)
    {
        last_volt = now;
        supply_voltage = (float)adc_mean_filter_convert(VOLT_ADC_CH, 8)
                         * VOLT_VREF / 4096.0f * VOLT_DIV_RATIO;
    }

    // 3) 周期上报(控制循环由中断完成，此处不阻塞控制)
    if (now - last_report >= REPORT_PERIOD_MS)
    {
        last_report = now;
        Report_Send();
    }
}
