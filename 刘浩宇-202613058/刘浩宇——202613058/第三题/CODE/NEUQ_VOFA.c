#include "NEUQ_lib_include.h"

//       CMD = 0x01 启动闭环             AA 55 01
//       CMD = 0x02 停止                 AA 55 02
//       CMD = 0x03 设置目标转速         AA 55 03
//       CMD = 0x04 设置单个 PID 参数    AA 55 04 <0/1/2>

#define FRAME_HEAD1    0xAA
#define FRAME_HEAD2    0x55

#define CMD_START       0x01      // 启动
#define CMD_STOP        0x02      // 停止
#define CMD_SET_TARGET  0x03      // 设目标转速
#define CMD_SET_PID     0x04      // 设单个 PID 参数

#define FRAME_MAX_DATA  16
static uint8_t frame_data[FRAME_MAX_DATA];

typedef enum{
    ST_WAIT_HEAD1 = 0, // 第一个帧头 0xAA
    ST_WAIT_HEAD2,     // 第二个帧头 0x55
    ST_WAIT_CMD,       // 命令
    ST_WAIT_DATA       // 数据
} RX_STATE;

static uint8_t     rx_fifo_buf[256];  // 存放接收字节的数组
static NEUQ_FIFO_t rx_fifo;           // FIFO 结构体

static uint8_t NEUQ_VOFA_CmdDataLen(uint8_t cmd){//获取命令长度
    switch(cmd){
        case CMD_SET_TARGET: 
            return 4;
        case CMD_SET_PID:   
            return 5;
        default:            
            return 0;
    }
}

void NEUQ_VOFA_INIT(void){
    NEUQ_FIFO_init(&rx_fifo, rx_fifo_buf, 256);  // 初始化接收队列
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 1);    // 开串口接收中断（优先级低于 10ms 控制中断）
}

void NEUQ_VOFA_RecvByte(uint8_t dat){
    NEUQ_FIFO_write_byte(&rx_fifo, dat);         // 中断里只做入队，解析放主循环
}

static void NEUQ_VOFA_HandleFrame(uint8_t frame_cmd, uint8_t *frame_data_in, uint8_t len){
    float f;
    switch(frame_cmd){
        case CMD_START:
            NEUQ_MOTOR_Start();
            break;

        case CMD_STOP:
            NEUQ_MOTOR_Stop();
            break;

        case CMD_SET_TARGET:
            if(len == 4){
                memcpy(&f, frame_data_in, 4);
                if(f == f){// 过滤 NaN
                    NEUQ_MOTOR_SetTarget(f);
                }
            }
            break;

        case CMD_SET_PID:
            // 数据 = 1 字节序号 + 4 字节 float；序号：0 = Kp，1 = Ki，2 = Kd
            if(len == 5){
                memcpy(&f, (frame_data_in + 1), 4);
                if(f == f){// 过滤 NaN
                    if(frame_data_in[0] == 0){
                        NEUQ_MOTOR_SetKp(f);
                    }else if(frame_data_in[0] == 1){
                        NEUQ_MOTOR_SetKi(f);
                    }else if(frame_data_in[0] == 2){
                        NEUQ_MOTOR_SetKd(f);
                    }
                }
            }
            break;
        default:
            break;
    }
}

static void NEUQ_VOFA_ParseByte(uint8_t message){
    static uint8_t  data_idx  = 0;      // 已收到多少个数据字节
    static uint8_t  data_need = 0;      // 本帧需要多少个数据字节
    static uint8_t  frame_cmd = 0;      // 本帧命令
    static RX_STATE rx_state  = ST_WAIT_HEAD1;

    switch(rx_state){
        case ST_WAIT_HEAD1:
            if(message == FRAME_HEAD1){
                rx_state = ST_WAIT_HEAD2;
            }
            break;

        case ST_WAIT_HEAD2:
            if(message == FRAME_HEAD2){
                rx_state = ST_WAIT_CMD;
            }else if(message == FRAME_HEAD1){
                //两HEAD1的情况，先停止
            }else{
                rx_state = ST_WAIT_HEAD1;
            }
            break;

        case ST_WAIT_CMD:
            frame_cmd = message;
            data_need = NEUQ_VOFA_CmdDataLen(frame_cmd);
            data_idx  = 0;
            if(data_need == 0){
                NEUQ_VOFA_HandleFrame(frame_cmd, NULL, 0);   // 无数据命令：立即执行
                rx_state = ST_WAIT_HEAD1;
            }else{
                rx_state = ST_WAIT_DATA;
            }
            break;

        case ST_WAIT_DATA:
            frame_data[data_idx] = message;
            data_idx++;
            if(data_idx >= data_need){
                NEUQ_VOFA_HandleFrame(frame_cmd, frame_data, data_need);
                rx_state = ST_WAIT_HEAD1;
            }
            break;

        default:
            rx_state = ST_WAIT_HEAD1;
            break;
    }
}

void NEUQ_VOFA_Process(void){//
    uint8_t message = 0;
    while(NEUQ_FIFO_read_byte(&rx_fifo, &message) == NEUQ_FIFO_OK){
        NEUQ_VOFA_ParseByte(message);
    }
}

#define VOFA_CH_NUM   5

void NEUQ_VOFA_Send(void){
    static uint32_t last_vofa = 0;
    uint32_t now = HAL_GetTick();
    float    ch[5];
    uint8_t  buf[5 * 4 + 4];
    uint8_t  i;

    if((uint32_t)(now - last_vofa) < 20){
        return;
    }
    last_vofa = now;

    ch[0] = NEUQ_MOTOR_GetTarget();   // 通道0：目标转速
    ch[1] = NEUQ_MOTOR_GetActual();   // 通道1：实际转速
    ch[2] = NEUQ_MOTOR_GetKp();       // 通道2：Kp
    ch[3] = NEUQ_MOTOR_GetKi();       // 通道3：Ki
    ch[4] = NEUQ_MOTOR_GetKd();       // 通道4：Kd

    for(i = 0; i < 5; i++){
        memcpy(&buf[i * 4], &ch[i], 4);
    }
    buf[5 * 4 + 0] = 0x00;  // JustFloat 帧尾
    buf[5 * 4 + 1] = 0x00;
    buf[5 * 4 + 2] = 0x80;
    buf[5 * 4 + 3] = 0x7F;

    NEUQ_Uart_write_data(UART_0, buf, sizeof(buf));
}
