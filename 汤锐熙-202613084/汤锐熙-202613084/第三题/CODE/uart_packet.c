#include "uart_packet.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include <string.h>

// ==================== 全局变量：接收到的参数存到这里 ====================
volatile uint8_t  run_flag   = 0;      // 运行标志：0=停止 1=运行
volatile int16_t  target_rpm = 0;      // 目标转速 (rpm)
volatile float    kp = 0.0f;           // PID 比例
volatile float    ki = 0.0f;           // PID 积分
volatile float    kd = 0.0f;           // PID 微分
volatile uint8_t  send_flag  = 0;      // 上报标志：1=需要发送一次（中断/解包置位，主循环清零）
volatile float    actual_rpm = 0.0f;    // 实际转速 (rpm)，中断写、主循环读
volatile int32_t  duty_out = 0;         // 调试：当前占空比（-10000~10000），中断写
// =====================================================================

// NEUQ_Uart.c 里实现了该函数但未在 NEUQ_Uart.h 中声明，这里手动声明
void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

// ==================== 接收环形缓冲区 ====================
#define RX_BUF_SIZE 256
static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
// =====================================================

// 帧解析状态机
typedef enum{
    WAIT_HEAD1,                          // 等帧头1 0xAA
    WAIT_HEAD2,                          // 等帧头2 0x55
    WAIT_CMD,                            // 等命令
    WAIT_LEN,                            // 等数据长度
    WAIT_DATA,                           // 收数据
    WAIT_SUM,                            // 等校验和
}FrameState;

static FrameState st  = WAIT_HEAD1;
static uint8_t  cmd  = 0;
static uint8_t  len  = 0;
static uint8_t  data[MAX_DATA_LEN];
static uint8_t  idx  = 0;
static float speed_send = 0.0f;
// 校验和：CMD + LEN + 数据逐字节求和，取低 8 位
static uint8_t calc_checksum(uint8_t cmd, uint8_t len, uint8_t *d){
    uint8_t sum = cmd + len;
    for(uint8_t i = 0; i < len; i++){
        sum += d[i];
    }
    return sum;
}

// 解出一帧后，把数据更新到全局变量
static void handle_frame(uint8_t cmd, uint8_t len, uint8_t *d){
    switch(cmd){
        case CMD_START:
            run_flag = 1;
            break;

        case CMD_STOP:
            run_flag = 0;
            break;

        case CMD_SPEED:
            if(len >= 4){
                float sp;
                memcpy(&sp, d, 4);                            // 小端 float（VOFA 滑块发的是 float）
                target_rpm = (int16_t)sp;                     // 取整存进 int16
                send_flag = 1;                                // 更新后回传一次
            }
            break;

        case CMD_PID:
            if(len >= 12){
                float kp_t, ki_t, kd_t;
                memcpy(&kp_t, d + 0, 4);                      // 小端 float
                memcpy(&ki_t, d + 4, 4);
                memcpy(&kd_t, d + 8, 4);
                if(run_flag == 0){                            // 题目要求：PID 只在停机时改
                    kp = kp_t;
                    ki = ki_t;
                    kd = kd_t;
                }
                send_flag = 1;                                // 更新后回传一次
            }
            break;

        default:
            break;
    }
}

// 初始化：使能 UART0 接收中断（调用一次）
void UartPacket_Init(void){
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 5);
}

// VOFA+ JustFloat 协议：无帧头，N 个 float（小端）+ 帧尾 00 00 80 7F
// 【调试版】7 通道：目标转速 / kp / ki / kd / 实际转速 / run_flag / duty
// VOFA 里通道数要填 7。调试完可改回 5 通道（删掉最后两个即可）
void UartPacket_VofaSend(void){
    float ch[7];
//    speed_send = speed_send*0.7 + actual_rpm*0.3;
    ch[0] = (float)target_rpm;
    ch[1] = kp;
    ch[2] = ki;
    ch[3] = kd;
    ch[4] = actual_rpm;
    ch[5] = (float)run_flag;    // 0=停机 1=运行
    ch[6] = (float)duty_out;    // 当前占空比（-10000~10000）

    NEUQ_Uart_write_data(UART_0, (uint8_t*)ch, sizeof(ch));   // 7×4 = 28 字节

    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};               // JustFloat 帧尾
    NEUQ_Uart_write_data(UART_0, tail, sizeof(tail));         // 4 字节
}

// 中断入口：把收到的字节塞进环形缓冲（由 TS_UART_0_RX_CALLBACK 调用）
void UartPacket_RxByte(uint8_t dat){
    uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
    if(next != rx_tail){          // 缓冲区未满才写入，满则丢弃
        rx_buf[rx_head] = dat;
        rx_head = next;
    }
}

// 主循环任务：把缓冲区里的字节喂给状态机解包（反复调用）
void UartPacket_Task(void){
    while(rx_tail != rx_head){
        uint8_t b = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

        switch(st){
            case WAIT_HEAD1:
                if(b == FRAME_HEAD1) st = WAIT_HEAD2;
                break;
            case WAIT_HEAD2:
                st = (b == FRAME_HEAD2) ? WAIT_CMD : WAIT_HEAD1;
                break;
            case WAIT_CMD:
                cmd = b;
                idx = 0;
                st = WAIT_LEN;
                break;
            case WAIT_LEN:
                len = b;
                if(len <= MAX_DATA_LEN){
                    // len=0（启动/停止）不能进 WAIT_DATA，否则会吃掉校验字节，
                    // 导致 idx 永远追不上 len=0，校验永远不执行
                    st = (len > 0) ? WAIT_DATA : WAIT_SUM;
                } else {
                    st = WAIT_HEAD1;
                }
                break;
            case WAIT_DATA:
                data[idx++] = b;
                if(idx == len) st = WAIT_SUM;
                break;
            case WAIT_SUM:
                // 转速/PID 两条命令放松校验（VOFA 滑块算不了自定义求和校验），
                // 启动/停止等其他命令仍严格校验，不受影响
                if(b == calc_checksum(cmd, len, data) || cmd == CMD_SPEED || cmd == CMD_PID){
                    handle_frame(cmd, len, data);             // 完整一帧 → 更新全局变量
                }
                st = WAIT_HEAD1;                              // 校验对错都回起点，继续收下一帧
                break;
            default:
                st = WAIT_HEAD1;
                break;
        }
    }
}
