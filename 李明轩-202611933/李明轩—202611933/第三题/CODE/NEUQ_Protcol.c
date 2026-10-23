#include "NEUQ_lib_include.h"

typedef enum{
    head,
    len,
    cmd,
    data,
    check,
    tail,
}Protocolstate;

// 中断里写、主循环读，加 volatile
volatile uint8_t protcol_update = 0;
volatile uint8_t protcol_cmd = 0;
volatile uint8_t protcol_data_len = 0;
volatile uint8_t protcol_data[PROTOCOL_DATA_MAX];

// 生效参数，供电机控制代码读取
volatile uint8_t motor_run = 0;           // 0停止 1运行
volatile int16_t motor_target_speed = 0;  // 目标转速 rpm
volatile float motor_kp = 3.0f;           // PID 初始值
volatile float motor_ki = 0.05f;
volatile float motor_kd = 0.0f;

static void protcol_send(uint8_t cmd, const uint8_t *data, uint8_t len){
    uint8_t buf[1 + 1 + 1 + PROTOCOL_DATA_MAX + 1 + 1];
    uint8_t idx = 0;
    uint8_t sum = (uint8_t)(len + cmd);

    buf[idx++] = PROTOCOL_HEAD;
    buf[idx++] = len;
    buf[idx++] = cmd;
    for(uint8_t i=0;i<len;i++){
        buf[idx++] = data[i];
        sum = (uint8_t)(sum + data[i]);
    }
    buf[idx++] = sum;
    buf[idx++] = PROTOCOL_TAIL;

    NEUQ_Uart_write_data(UART_0, buf, idx);
}

void NEUQ_Protcol_Init(void){
    NEUQ_Uart_init(UART_0, UART_BAUD_115200);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 0);
    protcol_update = 0;
    motor_run = 0;
    motor_target_speed = 0;
}

void NEUQ_Protcol_explain(uint8_t dat){
    static Protocolstate state = head;
    static uint8_t rx_len = 0;
    static uint8_t rx_cmd = 0;
    static uint8_t rx_index = 0;
    static uint8_t rx_check = 0;
    static uint8_t rx_data[PROTOCOL_DATA_MAX];

    switch(state){
        case head:{
            if(dat == PROTOCOL_HEAD){
                state = len;
            }
            break;
        }
        case len:{
            rx_len = dat;
            if(rx_len > PROTOCOL_DATA_MAX){
                state = head;   // 长度非法，重新同步
            }else{
                state = cmd;
            }
            break;
        }
        case cmd:{
            rx_cmd = dat;
            rx_index = 0;
            if(rx_len > 0){
                state = data;
            }else{
                state = check;
            }
            break;
        }
        case data:{
            rx_data[rx_index++] = dat;
            if(rx_index >= rx_len){
                state = check;
            }
            break;
        }
        case check:{
            rx_check = dat;
            state = tail;
            break;
        }
        case tail:{
            if(dat == PROTOCOL_TAIL){
                uint8_t sum = (uint8_t)(rx_len + rx_cmd);
                for(uint8_t i=0;i<rx_len;i++){
                    sum = (uint8_t)(sum + rx_data[i]);
                }
                if(rx_check == sum){
                    protcol_cmd = rx_cmd;
                    protcol_data_len = rx_len;
                    for(uint8_t i=0;i<rx_len;i++){
                        protcol_data[i] = rx_data[i];
                    }
                    protcol_update = 1;
                }
            }
            state = head;
            break;
        }
    }
}

void NEUQ_Protcol_task(void){
    if(protcol_update == 1){
        protcol_update = 0;
        switch(protcol_cmd){
            case CMD_START:{
                motor_run = 1;
                break;
            }
            case CMD_STOP:{
                motor_run = 0;
                motor_target_speed = 0;
                break;
            }
            case CMD_SET_SPEED:{
                if(protcol_data_len >= 2){
                    motor_target_speed = (int16_t)((uint16_t)protcol_data[0] | ((uint16_t)protcol_data[1] << 8));
                }
                break;
            }
            case CMD_SET_PID:{
                if(motor_run == 0 && protcol_data_len >= 12){
                    uint8_t buf[12];
                    for(uint8_t i=0;i<12;i++){
                        buf[i] = protcol_data[i];
                    }
                    float kp, ki, kd;
                    memcpy(&kp, buf + 0, 4);
                    memcpy(&ki, buf + 4, 4);
                    memcpy(&kd, buf + 8, 4);
                    motor_kp = kp;
                    motor_ki = ki;
                    motor_kd = kd;
                }
                break;
            }
            case CMD_QUERY:{
                NEUQ_Protcol_send_report((float)motor_target_speed, motor_actual_speed);
                break;
            }
            default:{
                break;
            }
        }
    }
}

void NEUQ_Protcol_send_report(float target, float actual){
    float kp = motor_kp;
    float ki = motor_ki;
    float kd = motor_kd;

    uint8_t data[20];
    memcpy(data + 0,  &target, 4);   // 目标转速
    memcpy(data + 4,  &actual, 4);   // 实际转速
    memcpy(data + 8,  &kp, 4);       // Kp
    memcpy(data + 12, &ki, 4);       // Ki
    memcpy(data + 16, &kd, 4);       // Kd
    protcol_send(CMD_REPORT, data, 20);
}
