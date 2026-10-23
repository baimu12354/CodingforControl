/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_Main
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

// ================== 串口协议 + VOFA+ 上传（原 NEUQ_VOFA 模块，已并入本文件） ==================
void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

#define FRAME_HEAD1    0xA0
#define FRAME_HEAD2    0xA1

#define CMD_START       0x01      // 启动
#define CMD_STOP        0x02      // 停止
#define CMD_SET_TARGET  0x03     // 设目标转速（float，符号 = 正转/反转）
#define CMD_SET_PID     0x04      // 设单个 PID 参数（1 字节序号 + float，序号 0/1/2 = Kp/Ki/Kd）

#define FRAME_MAX_DATA  16
static uint8_t frame_data[FRAME_MAX_DATA];

typedef enum{
    ST_WAIT_HEAD1 = 0, // 第一个帧头
    ST_WAIT_HEAD2,     // 第二个帧头
    ST_WAIT_CMD,       // 命令
    ST_WAIT_DATA       // 数据
} RX_STATE;

static uint8_t     rx_fifo_buf[256];  // 存放接收字节的数组
static NEUQ_FIFO_t rx_fifo;           // FIFO 结构体

static uint8_t NEUQ_VOFA_CmdDataLen(uint8_t cmd){//获取命令长度
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
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 1);    // 开串口接收中断（优先级低于 10ms 控制中断）
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
                //两HEAD1的情况
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
                memcpy(&f, frame_data_in, 4);          // 把 4 字节还原成 float
                if(f == f){                            // 过滤 NaN
                    NEUQ_MOTOR_SetTarget(f);
                }
            }
            break;

        case CMD_SET_PID:
            // 数据 = 1 字节序号 + 4 字节 float；序号：0 = Kp，1 = Ki，2 = Kd
            if(len == 5){
                memcpy(&f, (frame_data_in + 1), 4);
                if(f == f){                            // 过滤 NaN
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

#define VOFA_CH_NUM   5

void NEUQ_VOFA_Send(void){
    static uint32_t last_vofa = 0;
    uint32_t now = HAL_GetTick();
    float    ch[VOFA_CH_NUM];
    uint8_t  buf[VOFA_CH_NUM * 4 + 4];
    uint8_t  i;

    if((uint32_t)(now - last_vofa) < 20U){
        return;
    }
    last_vofa = now;

    ch[0] = NEUQ_MOTOR_GetTarget();   // 通道0：目标转速
    ch[1] = NEUQ_MOTOR_GetActual();   // 通道1：实际转速
    ch[2] = NEUQ_MOTOR_GetKp();       // 通道2：Kp
    ch[3] = NEUQ_MOTOR_GetKi();       // 通道3：Ki
    ch[4] = NEUQ_MOTOR_GetKd();       // 通道4：Kd

    for(i = 0; i < VOFA_CH_NUM; i++){
        memcpy(&buf[i * 4], &ch[i], 4);
    }
    buf[VOFA_CH_NUM * 4 + 0] = 0x00;  // JustFloat 帧尾
    buf[VOFA_CH_NUM * 4 + 1] = 0x00;
    buf[VOFA_CH_NUM * 4 + 2] = 0x80;
    buf[VOFA_CH_NUM * 4 + 3] = 0x7F;

    NEUQ_Uart_write_data(UART_0, buf, sizeof(buf));
}
// ==============================================================================

// **************************** 可调参数 ****************************
#define MOTOR_KP             1000.0f
#define MOTOR_KI             50.0f
#define MOTOR_KD             0.0f
// **************************************************************************************************

// **************************** 代码区域 ****************************
void User_Main(void){
   
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();

    NEUQ_MOTOR_INIT();      
    NEUQ_GPIO_init(PB0, GPIO_Mode_Output_PushPull, GPIO_High);
    NEUQ_Encoder_init(Encoder_TIM4, Encoder_TIM4_PhaseA_PB6, Encoder_TIM4_PhaseB_PB7);

    NEUQ_VOFA_INIT();
    NEUQ_MOTOR_SetPID(MOTOR_KP, MOTOR_KI, MOTOR_KD);

    NEUQ_PIT_init_ms(PIT_CH1,10);   
    while(1){
        NEUQ_VOFA_Process();        // 解析上位机下发的指令
        NEUQ_VOFA_Send();           // 20ms
    }
}
