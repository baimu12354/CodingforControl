/* VOFA+ 串口协议模块*/
#include "NEUQ_lib_include.h"  

// ------------------------------ 下行指令表 ------------------------------
//        CMD = 0x01 启动闭环                   AA 55 01
//        CMD = 0x02 停止                        AA 55 02
//        CMD = 0x03 设置目标转速               AA 55 03 <float>
//        CMD = 0x04 设置单个 PID 参数    AA 55 04 <0/1/2> <float>

void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

//帧格式常量
#define FRAME_HEAD1    0xAA   // 帧头第 1 字节
#define FRAME_HEAD2    0x55   // 帧头第 2 字节

#define CMD_START       0x01      // 启动
#define CMD_STOP        0x02      // 停止
#define CMD_SET_TARGET  0x03     // 设目标转速（float，符号 = 正转/反转）
#define CMD_SET_PID     0x04      // 设单个 PID 参数（1 字节序号 + float，序号 0/1/2 = Kp/Ki/Kd）

//接收缓冲
#define FRAME_MAX_DATA  16                 // 一帧数据区最大长度（当前最长是 5，留余量）
static uint8_t frame_data[FRAME_MAX_DATA]; // 正在接收的帧的数据区

// 接收状态机的 4 个状态
typedef enum{
    ST_WAIT_HEAD1 = 0, // 第一个帧头 0xAA
    ST_WAIT_HEAD2,     // 第二个帧头 0x55
    ST_WAIT_CMD,       // 命令
    ST_WAIT_DATA       // 数据
} RX_STATE;

//接收 FIFO
static uint8_t     rx_fifo_buf[256];  // 存放接收字节的数组
static NEUQ_FIFO_t rx_fifo;           // FIFO 结构体

// 查命令对应的数据长度（字节数），0 表示该命令没有数据区
static uint8_t NEUQ_VOFA_CmdDataLen(uint8_t cmd){
    switch(cmd){
        case CMD_SET_TARGET:
            return 4;   // float
        case CMD_SET_PID:
             return 5;   // 序号 + float
        default:
             return 0;   // 启动 / 停止 / 其它
    }
}

static void NEUQ_VOFA_ParseByte(uint8_t b);                                  // 处理单个字节
static void NEUQ_VOFA_HandleFrame(uint8_t cmd, uint8_t *data, uint8_t len);  // 执行一条完整指令

void NEUQ_VOFA_INIT(void){
    NEUQ_FIFO_init(&rx_fifo, rx_fifo_buf, 256);  // 初始化接收队列
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 1);    // 开串口接收中断
}

void NEUQ_VOFA_RecvByte(uint8_t dat){
    NEUQ_FIFO_write_byte(&rx_fifo, dat);         // 中断里只做入队，解析放主循环
}

void NEUQ_VOFA_Process(void){
    uint8_t message = 0;
    while(NEUQ_FIFO_read_byte(&rx_fifo, &message) == NEUQ_FIFO_OK){
        NEUQ_VOFA_ParseByte(message);
    }
}

//状态机解析
static void NEUQ_VOFA_ParseByte(uint8_t message){
    static uint8_t  data_idx  = 0;      // 已收到多少个数据字节
    static uint8_t  data_need = 0;      // 本帧需要多少个数据字节
    static uint8_t  frame_cmd = 0;      // 本帧命令
    static RX_STATE rx_state  = ST_WAIT_HEAD1;   // 初始状态

    switch(rx_state){
        case ST_WAIT_HEAD1:
            // 收到 0xAA -> 进入等第二帧头；收到其它字节 -> 丢弃（噪音/帧尾）
            if(message == FRAME_HEAD1){
                rx_state = ST_WAIT_HEAD2;
            }
            break;

        case ST_WAIT_HEAD2:
            if(message == FRAME_HEAD2){
                rx_state = ST_WAIT_CMD;         // 0xAA 0x55 齐全 -> 等命令
            }else if(message == FRAME_HEAD1){
            }else{
                rx_state = ST_WAIT_HEAD1;       // 其它字节 -> 整帧作废，回等帧头
            }
            break;

        case ST_WAIT_CMD:
            // 收到命令字节：查出本帧数据长度，决定是立即执行还是去收数据
            frame_cmd = message;                          // 记录命令
            data_need = NEUQ_VOFA_CmdDataLen(frame_cmd);  // 查表得到数据区长度
            data_idx  = 0;                                // 数据计数清零
            if(data_need == 0){
                NEUQ_VOFA_HandleFrame(frame_cmd, NULL, 0);   
                rx_state = ST_WAIT_HEAD1;                    // 回到等帧头
            }else{
                rx_state = ST_WAIT_DATA;                     // 有数据 -> 去收数据
            }
            break;

        case ST_WAIT_DATA:
            // 逐个收数据字节，收满 data_need 个就执行整帧
            frame_data[data_idx] = message;   // 存入数据区
            data_idx++;                       // 计数 +1
            if(data_idx >= data_need){        // 数据收齐
                NEUQ_VOFA_HandleFrame(frame_cmd, frame_data, data_need);  
                rx_state = ST_WAIT_HEAD1;     // 回到等帧头
            }
            break;

        default:
            rx_state = ST_WAIT_HEAD1;
            break;
    }
}

//指令执行
static void NEUQ_VOFA_HandleFrame(uint8_t frame_cmd, uint8_t *frame_data_in, uint8_t len){
    float f;   // 还原
    switch(frame_cmd){
        case CMD_START:
            NEUQ_MOTOR_Start();   // 启动闭环
            break;

        case CMD_STOP:
            NEUQ_MOTOR_Stop();    // 停止
            break;

        case CMD_SET_TARGET:
            if(len == 4){
                memcpy(&f, frame_data_in, 4);          // 把 4 字节还原成 float
                if(f == f){                            // 过滤 NaN
                    NEUQ_MOTOR_SetTarget(f);           // 设目标转速
                }
            }
            break;

        case CMD_SET_PID:
            if(len == 5){
                memcpy(&f, (frame_data_in + 1), 4);    // 跳过序号字节，还原 float
                if(f == f){                            // 过滤 NaN
                    if(frame_data_in[0] == 0){
                        NEUQ_MOTOR_SetKp(f);           // 序号 0 -> 设 Kp
                    }else if(frame_data_in[0] == 1){
                        NEUQ_MOTOR_SetKi(f);           // 序号 1 -> 设 Ki
                    }else if(frame_data_in[0] == 2){
                        NEUQ_MOTOR_SetKd(f);           // 序号 2 -> 设 Kd
                    }
                }
            }
            break;

        default:
            break; 
    }
}

//上行 JustFloat 上传
#define VOFA_CH_NUM   5   // 通道数：目标/实际/Kp/Ki/Kd

void NEUQ_VOFA_Send(void){
    static uint32_t last_vofa = 0;   // 上次上传的时刻（HAL_GetTick 的 ms 值）
    uint32_t now = HAL_GetTick();    // 当前时刻
    float    ch[VOFA_CH_NUM];        // 5 通道数据
    uint8_t  buf[VOFA_CH_NUM * 4 + 4];   // 发送缓冲：5×float + 4 字节帧尾 = 24 字节
    uint8_t  i;

    // 软节流：距上次上传不足 20ms 就直接返回（不用定时器，用主循环空转 + tick 判断）
    if((uint32_t)(now - last_vofa) < 20U){
        return;
    }
    last_vofa = now;   // 更新上次上传时刻

    ch[0] = NEUQ_MOTOR_GetTarget();   // 通道0：目标转速
    ch[1] = NEUQ_MOTOR_GetActual();   // 通道1：实际转速
    ch[2] = NEUQ_MOTOR_GetKp();       // 通道2：Kp
    ch[3] = NEUQ_MOTOR_GetKi();       // 通道3：Ki
    ch[4] = NEUQ_MOTOR_GetKd();       // 通道4：Kd

    // 把 float 按内存原始字节（小端）拷进发送缓冲：不经过 printf，4 字节原样传
    for(i = 0; i < VOFA_CH_NUM; i++){
        memcpy(&buf[i * 4], &ch[i], 4);
    }
    // JustFloat 帧尾：00 00 80 7F = 小端序 float 正无穷，VOFA+ 靠它识别一帧的结束
    buf[VOFA_CH_NUM * 4 + 0] = 0x00;  // JustFloat 帧尾
    buf[VOFA_CH_NUM * 4 + 1] = 0x00;
    buf[VOFA_CH_NUM * 4 + 2] = 0x80;
    buf[VOFA_CH_NUM * 4 + 3] = 0x7F;

    NEUQ_Uart_write_data(UART_0, buf, sizeof(buf));   // 整帧发出
}
