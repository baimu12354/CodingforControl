#include"NEUQ_lib_include.h"
#include "protocol.h"
#include "motor.h"

extern void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select,uint8_t priority);
#define PROTO_UART      UART_0
#define PROTO_BAUD      UART_BAUD_115200

#define FRAME_HEAD1     0xAA   //170
#define FRAME_HEAD2     0x55   //85

#define CMD_START       0x01    // 启动                     1
#define CMD_STOP        0x02    // 停止                     2 
#define CMD_SET_SPEED   0x03    //设目标速度 (4字节 float)   3
#define CMD_SET_PID     0x04    //12字节 3个float           4
#define CMD_SET_KP      0x05
#define CMD_SET_KI      0x06
#define CMD_SET_KD      0x07
typedef enum{
    ST_HEAD1,   //等待AA
    ST_HEAD2,   //等待55
    ST_CMD,     //接收命令字节
    ST_LEN,     //接收长度字节
    ST_DATA,    //接收数据字节
    ST_SUM,     //接收校验字节
}ParseState_t;

static ParseState_t s_state    = ST_HEAD1;
static uint8_t      s_cmd      = 0;
static uint8_t      s_len      = 0;
static uint8_t      s_data[32];     //数据缓冲, 最多 32 字节
static uint8_t      s_data_idx = 0; //数据索引
static uint8_t      s_sum      = 0; //异或校验累加

//初始化
void Protocol_Init(void){
    //初始化uart0
    NEUQ_Uart_init(PROTO_UART,PROTO_BAUD);
    //打开接收中断
    NEUQ_Uart_Enable_RX_Interrupt(PROTO_UART,5);
}
//中断里调用===================速度 PID调整==============================
static void handle_frame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    switch(cmd){
        case CMD_START:
        Motor_Start();
        break;

        case CMD_STOP:
        Motor_Stop();
        break;

        case CMD_SET_SPEED:{
            if(len!=4)return;
            float speed;
            memcpy(&speed,data,4);
            Motor_SetTargetSpeed(speed);
            if(!g_motor.running) Motor_Start();  // 首次设速自动启动，方便 VOFA+ 在线调参
            break;
        }

        case CMD_SET_PID:{
            if(len!=12)return;
            float kp,ki,kd;
            memcpy(&kp,data+0,4);
            memcpy(&ki,data+4,4);
            memcpy(&kd,data+8,4);
            Motor_SetPid(kp,ki,kd);
            break;
        }
        case CMD_SET_KP:{
            if(len!=4)return;
            float kp;
            memcpy(&kp,data,4);                         // 保持原有的 Ki 和 Kd 不变，只更新 Kp
            Motor_SetPid(kp, g_motor.ki, g_motor.kd); 
            break;
        }
        case CMD_SET_KI:{
            if(len!=4)return;
            float ki;
            memcpy(&ki,data,4);
            Motor_SetPid(g_motor.kp, ki, g_motor.kd);
            break;
        }
        case CMD_SET_KD:{
            if(len!=4)return;
            float kd;
            memcpy(&kd,data,4);
            Motor_SetPid(g_motor.kp, g_motor.ki, kd);
            break;
        }
    default :
    break;
    }
}
//=========================================接收数据============================================
void Protocol_RxByte(uint8_t byte){
    switch (s_state)
    {
    case ST_HEAD1:
        if(byte==FRAME_HEAD1)s_state=ST_HEAD2;
        break;
    case ST_HEAD2:
        if(byte==FRAME_HEAD2){
            s_state=ST_CMD;      //向下
        }else if(byte==FRAME_HEAD1){
            s_state=ST_HEAD2;    //仍是帧头，继续等 0x55
        }else{
            s_state=ST_HEAD1;    //重来
        }
        break;
        case ST_CMD:
            s_cmd  =byte;        //接收
            s_sum  =byte;        //接收

            s_state=ST_LEN;
            break;
        case ST_LEN:
        s_len=byte;
        if(s_len>sizeof(s_data)){
            s_state=ST_HEAD1;
            break;
        }
        s_sum^=byte;
        s_data_idx=0;
        s_state=(s_len==0)?ST_SUM:ST_DATA;
        break;
        case ST_DATA:
        s_data[s_data_idx++]=byte;
        s_sum^=byte;
        if(s_data_idx>=s_len)s_state=ST_SUM;
        break;
        case ST_SUM:
        if(byte==s_sum||s_cmd==CMD_SET_SPEED
                      ||s_cmd==CMD_SET_PID
                      ||s_cmd==CMD_SET_KP
                      ||s_cmd==CMD_SET_KI
                      ||s_cmd==CMD_SET_KD
                    ){
            handle_frame(s_cmd,s_data,s_len);
        }
        s_state=ST_HEAD1;
        break;
    }
}

//主循环回传
void Protocol_SendFeedback(void){
    static uint8_t pkt[24];
    memcpy(&pkt[0], &g_motor.target_speed, 4);
    memcpy(&pkt[4], &g_motor.actual_speed, 4);
    memcpy(&pkt[8], &g_motor.kp, 4);
    memcpy(&pkt[12],&g_motor.ki, 4);
    memcpy(&pkt[16],&g_motor.kd, 4);
    pkt[20] = 0x00;
    pkt[21] = 0x00;
    pkt[22] = 0x80;
    pkt[23] = 0x7F;
    NEUQ_Uart_write_data(PROTO_UART, pkt, 24);
}

// 主循环周期调用：按控制节拍每 20ms 回传一次
void NEUQ_Work_tick(void){
    static uint32_t last_tick = 0;
    uint32_t now = g_motor_tick_ms;   
    if (now - last_tick >= 20) {
        last_tick = now;
        Protocol_SendFeedback();
    }
}
