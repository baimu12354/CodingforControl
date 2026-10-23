#include "NEUQ_lib_include.h"
#include <math.h>

static NEUQ_FIFO_t rx_fifo;
static uint8_t rx_fifo_buf[256];
static volatile uint8_t rx_overflow = 0;
static ProtoState state = WAIT_HEAD;
static uint8_t rx_len = 0, rx_cmd = 0, rx_data_cnt = 0;
static uint8_t rx_data_buf[12];

void NEUQ_Protocol_Init(void)
{
    NEUQ_FIFO_init(&rx_fifo, rx_fifo_buf, sizeof(rx_fifo_buf));
    rx_overflow = 0;
    state = WAIT_HEAD;
    rx_len = rx_cmd = rx_data_cnt = 0;
}

void NEUQ_Protocol_Receive(uint8_t dat)
{
    if (NEUQ_FIFO_write_byte(&rx_fifo, dat) == NEUQ_FIFO_ERROR) {
        rx_overflow = 1;
    }
}

static void Protocol_Apply(void)
{
    float speed, kp, ki, kd, value;

    switch (rx_cmd) {
        case CMD_START_STOP:
            if (rx_data_buf[0] > 1) break;
            if (rx_data_buf[0] == 1) {
                if (motor_enable == 0) {
                    current_pwm = 0.0f;
                    Ipid_parameters_setting(&stra_speed_wheel_1,
                        stra_speed_wheel_1.Kp, stra_speed_wheel_1.Ki,
                        stra_speed_wheel_1.Kd);
                    motor_enable = 1;
                }
            } else {
                motor_enable = 0;
                current_pwm = 0.0f;
                Ipid_parameters_setting(&stra_speed_wheel_1,
                    stra_speed_wheel_1.Kp, stra_speed_wheel_1.Ki,
                    stra_speed_wheel_1.Kd);
                NEUQ_Motor_SetDuty(0);
            }
            break;

        case CMD_SET_SPEED:
            memcpy(&speed, rx_data_buf, sizeof(speed));
            if (isfinite(speed) && speed >= 0.0f) target_speed = speed;
            break;

        case CMD_SET_PID:
            if (motor_enable != 0) break;
            memcpy(&kp, &rx_data_buf[0], sizeof(kp));
            memcpy(&ki, &rx_data_buf[4], sizeof(ki));
            memcpy(&kd, &rx_data_buf[8], sizeof(kd));
            if (isfinite(kp) && isfinite(ki) && isfinite(kd) &&
                kp >= 0.0f && ki >= 0.0f && kd >= 0.0f) {
                Ipid_parameters_setting(&stra_speed_wheel_1, kp, ki, kd);
            }
            break;

        case CMD_SET_KP:
        case CMD_SET_KI:
        case CMD_SET_KD:
            if (motor_enable != 0) break;
            memcpy(&value, rx_data_buf, sizeof(value));
            if (!isfinite(value) || value < 0.0f) break;
            kp = stra_speed_wheel_1.Kp;
            ki = stra_speed_wheel_1.Ki;
            kd = stra_speed_wheel_1.Kd;
            if (rx_cmd == CMD_SET_KP) kp = value;
            else if (rx_cmd == CMD_SET_KI) ki = value;
            else kd = value;
            Ipid_parameters_setting(&stra_speed_wheel_1, kp, ki, kd);
            break;
    }
}

static uint8_t Protocol_CommandLength(uint8_t cmd)
{
    switch (cmd) {
        case CMD_START_STOP: return 1;
        case CMD_SET_SPEED:
        case CMD_SET_KP:
        case CMD_SET_KI:
        case CMD_SET_KD: return 4;
        case CMD_SET_PID: return 12;
        default: return 0;
    }
}

void NEUQ_Protocol_Analysis(uint8_t dat)
{
    switch (state) {
        case WAIT_HEAD:
            if (dat == PROTO_HEAD) state = WAIT_LEN;
            break;
        case WAIT_LEN:
            if (dat == 1 || dat == 4 || dat == 12) {
                rx_len = dat;
                state = WAIT_CMD;
            } else {
                state = (dat == PROTO_HEAD) ? WAIT_LEN : WAIT_HEAD;
            }
            break;
        case WAIT_CMD:
            if (Protocol_CommandLength(dat) == rx_len) {
                rx_cmd = dat;
                rx_data_cnt = 0;
                state = WAIT_DATA;
            } else {
                state = (dat == PROTO_HEAD) ? WAIT_LEN : WAIT_HEAD;
            }
            break;
        case WAIT_DATA:
            rx_data_buf[rx_data_cnt++] = dat;
            if (rx_data_cnt == rx_len) state = WAIT_TAIL;
            break;
        case WAIT_TAIL:
            if (dat == PROTO_TAIL) Protocol_Apply();
            state = (dat == PROTO_HEAD) ? WAIT_LEN : WAIT_HEAD;
            break;
    }
}

void NEUQ_Protocol_Task(void)
{
    uint8_t dat;
    uint32_t count;

    if (rx_overflow) {
        rx_overflow = 0;
        NEUQ_FIFO_drop(&rx_fifo, NEUQ_FIFO_get_used(&rx_fifo));
        state = WAIT_HEAD;
    }
    count = NEUQ_FIFO_get_used(&rx_fifo);
    while (count-- > 0 && NEUQ_FIFO_read_byte(&rx_fifo, &dat) == NEUQ_FIFO_OK) {
        NEUQ_Protocol_Analysis(dat);
    }
}

void NEUQ_Protocol_Report(void)
{
    float data[6];
    static uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

    if (telemetry_ready == 0) return;
    telemetry_ready = 0;
    data[0] = target_speed;
    data[1] = g_actual_speed;
    data[2] = stra_speed_wheel_1.Kp;
    data[3] = stra_speed_wheel_1.Ki;
    data[4] = stra_speed_wheel_1.Kd;
    data[5] = (float)motor_enable;
    NEUQ_Uart_write_data(UART_0, (uint8_t *)data, sizeof(data));
    NEUQ_Uart_write_data(UART_0, tail, sizeof(tail));
}
