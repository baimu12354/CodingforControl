#include "NEUQ_lib_include.h"


volatile int32_t target_speed = 0;// 目标转速
volatile int32_t actual_speed = 0;// 实际转速
volatile uint8_t motor_enable = 0;//使能，0停，1转
pid_value motor_pid;              //PID状态


// 串口接收 FIFO（串口中断写，主循环读）
NEUQ_FIFO_t uart_rx_fifo;
static uint8_t uart_rx_buf[256];


static void protocol_dispatch(uint8_t cmd, uint8_t *data, uint8_t len){
    switch(cmd){
    case CMD_START:
        pid_parameters_setting(&motor_pid, motor_pid.Kp, motor_pid.Ki, motor_pid.Kd); // 先清积分
        motor_enable = 1;                                                             // 再启动
        break;
    case CMD_STOP:
        motor_enable = 0;
        break;
    case CMD_SET_TARGET:
        if(len == 4){
            float v;
            memcpy(&v, data, 4);
            target_speed = (int32_t)v;      // 运行中改目标转速
        }
        break;
    case CMD_SET_PID:
        if(len == 12 && motor_enable == 0){ // PID 参数仅在停止状态下可修改
            float kp, ki, kd;
            memcpy(&kp, data + 0, 4);
            memcpy(&ki, data + 4, 4);
            memcpy(&kd, data + 8, 4);
            pid_parameters_setting(&motor_pid, kp, ki, kd);
        }
        break;
    case CMD_SET_KP:
        if(len == 4 && motor_enable == 0){
            float kp;
            memcpy(&kp, data, 4);
            pid_parameters_setting(&motor_pid, kp, motor_pid.Ki, motor_pid.Kd);
        }
        break;
    case CMD_SET_KI:
        if(len == 4 && motor_enable == 0){
            float ki;
            memcpy(&ki, data, 4);
            pid_parameters_setting(&motor_pid, motor_pid.Kp, ki, motor_pid.Kd);
        }
        break;
    case CMD_SET_KD:
        if(len == 4 && motor_enable == 0){
            float kd;
            memcpy(&kd, data, 4);
            pid_parameters_setting(&motor_pid, motor_pid.Kp, motor_pid.Ki, kd);
        }
        break;
    }
}


typedef enum { S_HEAD1, S_HEAD2, S_CMD, S_LEN, S_DATA } rx_state_t;

static rx_state_t rx_state = S_HEAD1;
static uint8_t rx_cmd, rx_len, rx_idx;
static uint8_t rx_data[FRAME_DATA_MAX];

void protocol_parse_byte(uint8_t b){
    switch(rx_state){
    case S_HEAD1:
        if(b == FRAME_HEAD1) rx_state = S_HEAD2;
        break;
    case S_HEAD2:
        if(b == FRAME_HEAD2)      rx_state = S_CMD;    // 完整帧头
        else if(b == FRAME_HEAD1) rx_state = S_HEAD2;  // 处理 AA AA 粘连
        else                      rx_state = S_HEAD1;
        break;
    case S_CMD:
        rx_cmd = b;
        rx_state = S_LEN;
        break;
    case S_LEN:
        rx_len = b;
        if(rx_len > FRAME_DATA_MAX){            // 非法长度，丢弃
            rx_state = S_HEAD1;
        }else if(rx_len == 0){                  // 无数据，直接执行
            protocol_dispatch(rx_cmd, rx_data, 0);
            rx_state = S_HEAD1;
        }else{
            rx_idx = 0;
            rx_state = S_DATA;
        }
        break;
    case S_DATA:
        rx_data[rx_idx++] = b;
        if(rx_idx >= rx_len){                   // 收满，执行
            protocol_dispatch(rx_cmd, rx_data, rx_len);
            rx_state = S_HEAD1;
        }
        break;
    }
}


void protocol_send_echo(void){
    static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};   // JustFloat 帧尾

    float data[5];
    data[0] = (float)target_speed;   // 目标转速（rpm）
    data[1] = (float)actual_speed;   // 实际转速（rpm）
    data[2] = motor_pid.Kp;
    data[3] = motor_pid.Ki;
    data[4] = motor_pid.Kd;

    NEUQ_Uart_write_data(UART_0, (uint8_t*)data, sizeof(data));
    NEUQ_Uart_write_data(UART_0, (uint8_t*)tail, sizeof(tail));
}


void NEUQ_Pack_init(void){
    NEUQ_FIFO_init(&uart_rx_fifo, uart_rx_buf, sizeof(uart_rx_buf));
    rx_state = S_HEAD1;
}
