#ifndef __FRAME_PARSE_H
#define __FRAME_PARSE_H

#include <stdint.h>

/* ==================== 诊断结构体 ==================== */
typedef struct {
    uint32_t rx_byte_count;
    uint32_t frame_ok_count;
    uint32_t frame_err_count;
    uint32_t fifo_overflow;
    uint8_t  parse_state;
    uint8_t  last_frame_type;
    uint32_t last_parse_ms;
} frame_diag_t;

/* ==================== 外部变量 ==================== */
extern uint8_t sys_running;
extern frame_diag_t g_frame_diag;

/* ==================== 函数声明 ==================== */
void frame_parse_poll(void);
void param_apply_poll(void);
void send_status_justfloat(void);
void frame_diag_reset(void);
uint8_t frame_diag_get_state(void);

#endif