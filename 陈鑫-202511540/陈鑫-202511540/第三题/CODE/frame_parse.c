#include "NEUQ_lib_include.h"

extern NEUQ_FIFO_t uart_rx_fifo;
extern uint32_t get_uart_rx_overflow(void);

/* ==================== 全局变量 ==================== */
uint8_t sys_running = 0;

/* pending 缓冲 */
static int32_t pending_target = 0;
static float   pending_kp = 0, pending_ki = 0, pending_kd = 0;
static uint8_t pending_target_flag = 0;
#define PID_PEND_KP   0x01
#define PID_PEND_KI   0x02
#define PID_PEND_KD   0x04
static uint8_t pending_pid_mask = 0;

/* JustFloat 帧尾 */
static const uint8_t jf_tail[4] = {0x00, 0x00, 0x80, 0x7F};

/* ==================== 诊断 ==================== */
frame_diag_t g_frame_diag = {0};

/* ==================== 校验 ==================== */
static uint8_t xor_checksum(const uint8_t *data, uint16_t len) {
    uint8_t x = 0;
    for (uint16_t i = 0; i < len; i++) x ^= data[i];
    return x;
}

/* ==================== 状态机 ==================== */
typedef enum {
    PARSE_WAIT_H1, PARSE_WAIT_H2, PARSE_WAIT_TYPE, PARSE_WAIT_LEN,
    PARSE_PAYLOAD, PARSE_WAIT_CRC_L, PARSE_WAIT_CRC_H,
    PARSE_WAIT_T1, PARSE_WAIT_T2
} parse_state_t;

static parse_state_t parse_state = PARSE_WAIT_H1;
static uint8_t  frame_type;
static uint8_t  frame_len;
static uint8_t  frame_payload[32];
static uint8_t  payload_idx;
static uint16_t frame_crc;

/* ==================== JustFloat 回传 ==================== */
void send_status_justfloat(void) {
#if JF_DEBUG_MODE
    if (g_frame_diag.rx_byte_count > 65536) {
        frame_diag_reset();
    }
#endif

    float tx[JF_CHANNEL_NUM];

    tx[0] = (float)spd_aim;
    tx[1] = (float)spd_now;
    tx[2] = pid_kp;
    tx[3] = pid_ki;
    tx[4] = pid_kd;
    tx[5] = (float)sys_running;

#if JF_DEBUG_MODE
    tx[6]  = (float)g_frame_diag.rx_byte_count;
    tx[7]  = (float)g_frame_diag.frame_ok_count;
    tx[8]  = (float)g_frame_diag.frame_err_count;
    tx[9]  = (float)g_frame_diag.fifo_overflow;
    tx[10] = (float)g_frame_diag.parse_state;
    tx[11] = (float)g_frame_diag.last_frame_type;
    tx[12] = (float)duty_now;
    tx[13] = (float)enc_total_cnt;
    tx[14] = (float)step_rise_ms;
#endif

    NEUQ_Uart_write_data(UART_0, (uint8_t *)tx, JF_CHANNEL_NUM * 4);
    NEUQ_Uart_write_data(UART_0, (uint8_t *)jf_tail, 4);
}


/* ==================== 载荷解析 ==================== */
static uint8_t payload_is_ascii_number(const uint8_t *p, uint8_t len)
{
    uint8_t i;
    if (len == 0 || len > 16) return 0;
    for (i = 0; i < len; i++) {
        uint8_t c = p[i];
        if (!((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == ' ')) {
            return 0;
        }
    }
    return 1;
}

static uint8_t decode_float_payload(const uint8_t *p, uint8_t len, float *out)
{
    if (payload_is_ascii_number(p, len)) {
        float v = 0.0f, scale = 1.0f;
        uint8_t i = 0, neg = 0, dot = 0;
        while (i < len && p[i] == ' ') i++;
        if (i < len && (p[i] == '-' || p[i] == '+')) { neg = (p[i] == '-'); i++; }
        for (; i < len; i++) {
            uint8_t c = p[i];
            if (c == '.') { dot = 1; continue; }
            if (c < '0' || c > '9') break;
            if (!dot) v = v * 10.0f + (float)(c - '0');
            else { scale *= 10.0f; v += (float)(c - '0') / scale; }
        }
        if (neg) v = -v;
        if (v > -10000.0f && v < 10000.0f) { *out = v; return 1; }
        return 0;
    }

    if (len >= 4) {
        float v;
        memcpy(&v, p, 4);
        if (v == v && v > -10000.0f && v < 10000.0f) {
            *out = v;
            return 1;
        }
        return 0;
    }

    if (len >= 2) {
        *out = (float)(int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        return 1;
    }
    return 0;
}

static uint8_t decode_target(const uint8_t *p, uint8_t len, int32_t *out)
{
    float v;
    if (!decode_float_payload(p, len, &v)) return 0;
    *out = (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
    return 1;
}

/* ==================== 帧解析 ==================== */
void frame_parse_poll(void) {
    g_frame_diag.fifo_overflow = get_uart_rx_overflow();

    uint8_t byte;
    while (NEUQ_FIFO_read_byte(&uart_rx_fifo, &byte) == NEUQ_FIFO_OK) {
        g_frame_diag.rx_byte_count++;

        switch (parse_state) {
        case PARSE_WAIT_H1:
            if (byte == FRAME_HEADER_1) parse_state = PARSE_WAIT_H2;
            break;
        case PARSE_WAIT_H2:
            if (byte == FRAME_HEADER_2) parse_state = PARSE_WAIT_TYPE;
            else parse_state = PARSE_WAIT_H1;
            break;
        case PARSE_WAIT_TYPE:
            frame_type = byte;
            parse_state = PARSE_WAIT_LEN;
            break;
        case PARSE_WAIT_LEN:
            frame_len = byte;
            if (frame_len > 32) { parse_state = PARSE_WAIT_H1; break; }
            payload_idx = 0;
            parse_state = (frame_len == 0) ? PARSE_WAIT_CRC_L : PARSE_PAYLOAD;
            break;
        case PARSE_PAYLOAD:
            frame_payload[payload_idx++] = byte;
            if (payload_idx >= frame_len) parse_state = PARSE_WAIT_CRC_L;
            break;
        case PARSE_WAIT_CRC_L:
            frame_crc = byte;
            parse_state = PARSE_WAIT_CRC_H;
            break;
        case PARSE_WAIT_CRC_H:
            frame_crc |= (byte << 8);
            parse_state = PARSE_WAIT_T1;
            break;
        case PARSE_WAIT_T1:
            if (byte == FRAME_TAIL_1) parse_state = PARSE_WAIT_T2;
            else parse_state = PARSE_WAIT_H1;
            break;
        case PARSE_WAIT_T2:
            if (byte == FRAME_TAIL_2) {
                if (1) {
                    g_frame_diag.frame_ok_count++;
                    g_frame_diag.last_frame_type = frame_type;
                    g_frame_diag.last_parse_ms = NEUQ_Get_Timer_Value_ms();

                    switch (frame_type) {
                    case FRAME_CMD_START:
                        sys_running = 1;
                        motor_pid_reset();
                        break;
                    case FRAME_CMD_STOP:
                        sys_running = 0;
                        motor_pid_reset();
                        motor_setduty(0);
                        break;
                    case FRAME_CMD_SET_TARGET:
                        {
                            int32_t tgt;
                            if (decode_target(frame_payload, frame_len, &tgt)) {
                                pending_target = tgt;
                                pending_target_flag = 1;
                            }
                        }
                        break;
                    case FRAME_CMD_SET_PID:
                        if (frame_len >= 12) {
                            pending_kp = *(float *)(frame_payload + 0);
                            pending_ki = *(float *)(frame_payload + 4);
                            pending_kd = *(float *)(frame_payload + 8);
                            pending_pid_mask |= (PID_PEND_KP | PID_PEND_KI | PID_PEND_KD);
                        }
                        break;
                    case FRAME_CMD_SET_KP:
                    case FRAME_CMD_SET_KI:
                    case FRAME_CMD_SET_KD:
                        {
                            float v;
                            if (decode_float_payload(frame_payload, frame_len, &v)) {
                                switch (frame_type) {
                                case FRAME_CMD_SET_KP:
                                    pending_kp = v;
                                    pending_pid_mask |= PID_PEND_KP;
                                    break;
                                case FRAME_CMD_SET_KI:
                                    pending_ki = v;
                                    pending_pid_mask |= PID_PEND_KI;
                                    break;
                                default:
                                    pending_kd = v;
                                    pending_pid_mask |= PID_PEND_KD;
                                    break;
                                }
                            }
                        }
                        break;
                    case FRAME_CMD_ENC_CLR:
                        enc_total_cnt = 0;
                        break;
                    case FRAME_CMD_QUERY:
                        send_status_justfloat();
                        break;
                    default:
                        break;
                    }
                } else {
                    g_frame_diag.frame_err_count++;
                }
            } else {
                g_frame_diag.frame_err_count++;
            }
            parse_state = PARSE_WAIT_H1;
            break;
        }
        g_frame_diag.parse_state = (uint8_t)parse_state;
    }
}

/* ==================== 参数应用 ==================== */
void param_apply_poll(void) {
    if (pending_target_flag) {
        spd_aim = pending_target;
        pending_target_flag = 0;
    }
    if (pending_pid_mask) {
        uint8_t allow = 1;
#if (PID_ONLINE_TUNE == 0)
        allow = (sys_running == 0);
#endif
        if (allow) {
            __disable_irq();
            if (pending_pid_mask & PID_PEND_KP) pid_kp = pending_kp;
            if (pending_pid_mask & PID_PEND_KI) pid_ki = pending_ki;
            if (pending_pid_mask & PID_PEND_KD) pid_kd = pending_kd;
            __enable_irq();
        }
        pending_pid_mask = 0;
    }
}

void frame_diag_reset(void) {
    memset(&g_frame_diag, 0, sizeof(frame_diag_t));
}

uint8_t frame_diag_get_state(void) {
    return g_frame_diag.parse_state;
}