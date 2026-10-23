/*********************************************************************************************************************
* 文件名称          NEUQ_Motor
* 功能描述          第三题核心：测速、速度环 PID、PWM 驱动、字节包协议、VOFA 上传
* 结构说明          控制循环在 PIT 定周期中断（Motor_Control_ISR），串口解析与上传在主循环（Motor_Task）
********************************************************************************************************************/
#include "NEUQ_Motor.h"


static volatile uint8_t g_running    = 0;
static volatile float   g_target_rpm = 0.0f;
static volatile float   g_actual_rpm = 0.0f;
static volatile float   g_pwm_out    = 0.0f;
static volatile float   g_Kp = DEFAULT_KP;
static volatile float   g_Ki = DEFAULT_KI;
static volatile float   g_Kd = DEFAULT_KD;
static volatile uint8_t g_pid_reset  = 0;


static float g_integral = 0.0f;
static float g_prev_meas = 0.0f;


static int32_t g_last_cnt  = 0;
static float   g_rpm_scale = 1.0f;

/* 串口接收 FIFO */
#define RX_FIFO_SIZE 256
static uint8_t     g_rx_buf[RX_FIFO_SIZE];
static NEUQ_FIFO_t g_rx_fifo;

/* 帧解析状态机 */
typedef enum { S_HDR1, S_HDR2, S_CMD, S_LEN, S_DATA, S_SUM, S_TAIL } FrameState_t;
static FrameState_t g_state = S_HDR1;
static uint8_t g_cmd = 0;
static uint8_t g_len = 0;
static uint8_t g_idx = 0;
static uint8_t g_frame[FRAME_MAX_LEN];


static uint32_t g_last_justfloat = 0;
static uint32_t g_last_echo = 0;

static void Motor_ProcessByte(uint8_t b);


static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void put_float(uint8_t *buf, float f) { memcpy(buf, &f, 4); }
static float get_float(const uint8_t *buf) { float f; memcpy(&f, buf, 4); return f; }


static void Motor_ApplyOutput(float out)
{
    int32_t duty;
    GPIO_Level dir;

    if (out >= 0.0f) {
        dir  = DIR_FORWARD_LEVEL;
        duty = (int32_t)(out + 0.5f);
    } else {
        dir  = DIR_REVERSE_LEVEL;
        duty = (int32_t)(-out + 0.5f);
    }
    if (duty > PWM_MAX) duty = PWM_MAX;
    if (duty < 0) duty = 0;

    NEUQ_GPIO_write(DIR_PIN, dir);
    NEUQ_PWM_set_duty(PWM_PORT, (uint32_t)duty);
}

static void Motor_SendJustFloat(void)
{
    uint8_t buf[4 * 4 + 4];
    float target = g_target_rpm;
    float actual = g_actual_rpm;
    float err    = target - actual;
    float pwm    = g_pwm_out;

    put_float(buf + 0,  target);
    put_float(buf + 4,  actual);
    put_float(buf + 8,  err);
    put_float(buf + 12, pwm);
    buf[16] = 0x00; buf[17] = 0x00; buf[18] = 0x80; buf[19] = 0x7F;

    NEUQ_Uart_write_data(UART_0, buf, sizeof(buf));
}


static void Motor_SendEcho(void)
{
    uint8_t payload[20];
    uint8_t buf[4 + 20 + 2];

    put_float(payload + 0,  g_target_rpm);
    put_float(payload + 4,  g_Kp);
    put_float(payload + 8,  g_Ki);
    put_float(payload + 12, g_Kd);
    put_float(payload + 16, g_actual_rpm);

    uint8_t sum = CMD_ECHO + 20;
    for (int i = 0; i < 20; i++) sum += payload[i];

    buf[0] = FRAME_HDR1;
    buf[1] = FRAME_HDR2;
    buf[2] = CMD_ECHO;
    buf[3] = 20;
    memcpy(buf + 4, payload, 20);
    buf[24] = sum;
    buf[25] = FRAME_TAIL;

    NEUQ_Uart_write_data(UART_0, buf, sizeof(buf));
}


#if ASCII_DEBUG_CMD_ENABLE
/* ASCII 调试命令：start / stop / speed 1500 / pid 2.0 0.2 0 / kp 1.5 / ki 0.1 / kd 0.01 */
static char    g_txt[40];
static uint8_t g_txt_len = 0;
static uint32_t g_txt_last_ms = 0;

static const char *txt_skip_space(const char *p)
{
    while (*p == ' ') p++;
    return p;
}

static uint8_t txt_parse_float(const char **pp, float *out)
{
    const char *p = txt_skip_space(*pp);
    float sign = 1.0f, val = 0.0f, frac = 0.0f, scale = 1.0f;
    uint8_t digits = 0;

    if (*p == '-') { sign = -1.0f; p++; }
    else if (*p == '+') { p++; }
    while (*p >= '0' && *p <= '9') { val = val * 10.0f + (float)(*p - '0'); p++; digits++; }
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            scale *= 0.1f;
            frac  += (float)(*p - '0') * scale;
            p++; digits++;
        }
    }
    if (!digits) return 0;
    *out = sign * (val + frac);
    *pp  = p;
    return 1;
}

static uint8_t txt_kw(const char **pp, const char *kw)
{
    const char *p = *pp;
    uint8_t n = 0;
    while (kw[n] != '\0') {
        if (p[n] != kw[n]) return 0;
        n++;
    }
    if (p[n] != '\0' && p[n] != ' ') return 0;
    *pp = p + n;
    return 1;
}

static uint8_t hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    return 0xFF;
}

/* 兼容 VOFA 按钮以“文本”方式发送十六进制帧（如 "AA 55 01 00 01 ED"）：
   解析为字节后走同一套二进制帧状态机，含校验与回显 */
static void Motor_HandleHexText(const char *line)
{
    const char *p = line;
    uint8_t hi, lo;

    while (*p) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0') break;
        hi = hex_nibble(p[0]);
        lo = hex_nibble(p[1]);
        if (hi == 0xFF || lo == 0xFF) return;
        Motor_ProcessByte((uint8_t)((hi << 4) | lo));
        p += 2;
    }
}

static void Motor_HandleText(const char *line)
{
    const char *p = line;
    float a, b, c;

    if (txt_kw(&p, "start")) {
        g_running   = 1;
        g_pid_reset = 1;
        if (g_target_rpm == 0.0f) g_target_rpm = DEFAULT_TARGET_RPM;
    } else if (txt_kw(&p, "stop")) {
        g_running    = 0;
        g_target_rpm = 0.0f;
        g_pid_reset  = 1;
    } else if (txt_kw(&p, "speed")) {
        if (!txt_parse_float(&p, &a)) return;
        g_target_rpm = clampf(a, -MAX_TARGET_RPM, MAX_TARGET_RPM);
    } else if (txt_kw(&p, "pid")) {
        if (!txt_parse_float(&p, &a) || !txt_parse_float(&p, &b) || !txt_parse_float(&p, &c)) return;
        if (!g_running) {
            if (a >= 0.0f) g_Kp = a;
            if (b >= 0.0f) g_Ki = b;
            if (c >= 0.0f) g_Kd = c;
        }
    } else if (txt_kw(&p, "kp")) {
        if (!txt_parse_float(&p, &a)) return;
        if (!g_running && a >= 0.0f) g_Kp = a;
    } else if (txt_kw(&p, "ki")) {
        if (!txt_parse_float(&p, &a)) return;
        if (!g_running && a >= 0.0f) g_Ki = a;
    } else if (txt_kw(&p, "kd")) {
        if (!txt_parse_float(&p, &a)) return;
        if (!g_running && a >= 0.0f) g_Kd = a;
    } else {
        Motor_HandleHexText(line);
        return;
    }

    Motor_SendEcho();
}

static void Motor_FlushText(void)
{
    if (g_txt_len == 0) return;
    g_txt[g_txt_len] = '\0';
    Motor_HandleText(g_txt);
    g_txt_len = 0;
}

static void Motor_TextByte(uint8_t b)
{
    if (b == FRAME_HDR1) { g_txt_len = 0; return; }   /* 二进制帧开始，丢弃未完成文本行 */
    if (b == '\r' || b == '\n' || b == '\0') {
        Motor_FlushText();
        return;
    }
    if (b >= 0x20 && b <= 0x7E) {
        if (g_txt_len < sizeof(g_txt) - 1) {
            g_txt[g_txt_len++] = (char)b;
            g_txt_last_ms = NEUQ_Get_Timer_Value_ms(); /* 记录最近字节，供无结束符时超时补发 */
        } else {
            g_txt_len = 0;                             /* 行过长，丢弃 */
        }
    } else {
        g_txt_len = 0;                                 /* 非文本字节，丢弃当前行 */
    }
}
#endif


static void Motor_HandleFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd) {
        case CMD_START:
            if (len == 0) {
                g_running   = 1;
                g_pid_reset = 1;
                if (g_target_rpm == 0.0f) g_target_rpm = DEFAULT_TARGET_RPM;
            }
            break;

        case CMD_STOP:
            if (len == 0) {
                g_running    = 0;
                g_target_rpm = 0.0f;
                g_pid_reset  = 1;
            }
            break;

        case CMD_SET_TARGET:
            if (len == 4) {
                float t = get_float(payload);
                g_target_rpm = clampf(t, -MAX_TARGET_RPM, MAX_TARGET_RPM);
            }
            break;

        case CMD_SET_PID:

            if (len == 12 && g_running == 0) {
                float kp = get_float(payload);
                float ki = get_float(payload + 4);
                float kd = get_float(payload + 8);
                if (kp >= 0.0f) g_Kp = kp;
                if (ki >= 0.0f) g_Ki = ki;
                if (kd >= 0.0f) g_Kd = kd;
            }
            break;

        default:
            break;
    }


    Motor_SendEcho();
}

static void Motor_ProcessByte(uint8_t b)
{
    switch (g_state) {
        case S_HDR1:
            if (b == FRAME_HDR1) g_state = S_HDR2;
            break;

        case S_HDR2:
            if (b == FRAME_HDR2)      g_state = S_CMD;
            else if (b == FRAME_HDR1) g_state = S_HDR2;
            else                      g_state = S_HDR1;
            break;

        case S_CMD:
            g_cmd   = b;
            g_state = S_LEN;
            break;

        case S_LEN:
            g_len = b;
            if (g_len > FRAME_MAX_LEN) {
                g_state = S_HDR1;                        
            } else {
                g_idx   = 0;
                g_state = (g_len == 0) ? S_SUM : S_DATA;
            }
            break;

        case S_DATA:
            g_frame[g_idx++] = b;
            if (g_idx >= g_len) g_state = S_SUM;
            break;

        case S_SUM: {
            uint8_t sum = g_cmd + g_len;
            for (uint8_t i = 0; i < g_len; i++) sum += g_frame[i];
            g_state = (b == sum) ? S_TAIL : S_HDR1;
            break;
        }

        case S_TAIL:
            if (b == FRAME_TAIL) {
                Motor_HandleFrame(g_cmd, g_frame, g_len);
            }
            g_state = S_HDR1;
            break;

        default:
            g_state = S_HDR1;
            break;
    }
}

//初始化
void Motor_Init(void)
{
    g_running    = 0;
    g_target_rpm = 0.0f;
    g_actual_rpm = 0.0f;
    g_pwm_out    = 0.0f;
    g_integral   = 0.0f;
    g_prev_meas  = 0.0f;
    g_last_cnt   = 0;
    g_pid_reset  = 0;

    g_rpm_scale = ENC_SIGN * 60.0f / (4.0f * ENCODER_PPR * ((float)CONTROL_PERIOD_MS / 1000.0f));

    NEUQ_Timer_init();   /* 启动 TIM13：Motor_Task 的限流计时依赖 NEUQ_Get_Timer_Value_ms */

    NEUQ_FIFO_init(&g_rx_fifo, g_rx_buf, RX_FIFO_SIZE);

    NEUQ_Encoder_init(ENC_DEV, ENC_PHASE_A, ENC_PHASE_B);
    NEUQ_GPIO_init(DIR_PIN, GPIO_Mode_Output_PushPull, DIR_REVERSE_LEVEL);
    NEUQ_PWM_init(PWM_PORT, PWM_FREQ_HZ, 0);

    Motor_ApplyOutput(0.0f);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 5);
    NEUQ_PIT_init_ms(PIT_CH1, CONTROL_PERIOD_MS);
}

void Motor_Control_ISR(void)
{

    int32_t cnt   = NEUQ_Encoder_Read(ENC_DEV);
    int32_t delta = cnt - g_last_cnt;
    g_last_cnt    = cnt;

    float inst_rpm = (float)delta * g_rpm_scale;
    g_actual_rpm += SPEED_EMA_ALPHA * (inst_rpm - g_actual_rpm);


    if (g_pid_reset) {
        g_integral  = 0.0f;
        g_prev_meas = g_actual_rpm;
        g_pid_reset = 0;
    }

    if (g_running) {
        float err = g_target_rpm - g_actual_rpm;
        g_integral += err;
        g_integral = clampf(g_integral, -PID_I_MAX, PID_I_MAX);

        float d_meas = g_actual_rpm - g_prev_meas;
        float out = g_Kp * err + g_Ki * g_integral - g_Kd * d_meas;
        out = clampf(out, -(float)PWM_MAX, (float)PWM_MAX);

        g_pwm_out    = out;
        g_prev_meas  = g_actual_rpm;
        Motor_ApplyOutput(out);
    } else {
        g_integral  = 0.0f;
        g_prev_meas = g_actual_rpm;
        g_pwm_out   = 0.0f;
        Motor_ApplyOutput(0.0f);
    }
}

void Motor_OnRxByte(uint8_t dat)
{
    NEUQ_FIFO_write_byte(&g_rx_fifo, dat);
}

void Motor_Task(void)
{

    uint8_t b;
    while (NEUQ_FIFO_read_byte(&g_rx_fifo, &b) == NEUQ_FIFO_OK) {
#if ASCII_DEBUG_CMD_ENABLE
        Motor_TextByte(b);
#endif
        Motor_ProcessByte(b);
    }


    uint32_t now = NEUQ_Get_Timer_Value_ms();

#if ASCII_DEBUG_CMD_ENABLE
    if (g_txt_len > 0 && (now - g_txt_last_ms) >= 50) {
        Motor_FlushText();   /* 无结束符时，50ms 无新字节视为一行结束 */
    }
#endif

    if ((now - g_last_justfloat) >= 10) {
        g_last_justfloat = now;
        Motor_SendJustFloat();
    }
    if ((now - g_last_echo) >= 500) {
        g_last_echo = now;
        Motor_SendEcho();
    }
}
