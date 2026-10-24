#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

/*------------------ 帧格式常量 ------------------*/
#define PROTOCOL_HEAD1       0xAA          // 帧头1
#define PROTOCOL_HEAD2       0x55          // 帧头2
#define PROTOCOL_DATA_MAX    16            // 单帧数据区最大长度（字节）

/*------------------ 下行命令字（PC -> MCU） ------------------*/
#define CMD_START            0x01          // 启动电机，无数据
#define CMD_STOP             0x02          // 停止电机，无数据
#define CMD_SET_TARGET       0x03          // 设目标转速，DATA = int32_t（编码器计数/秒，小端）
#define CMD_SET_PID          0x04          // 设PID参数，DATA = float kp + float ki + float kd（小端）

/*------------------ 接收状态机状态 ------------------*/
typedef enum {
    RX_STATE_IDLE = 0,      // 等待帧头1
    RX_STATE_HEAD1,         // 已收帧头1，等帧头2
    RX_STATE_CMD,           // 已收帧头2，等命令字
    RX_STATE_LEN,           // 已收命令字，等长度
    RX_STATE_DATA,          // 正在收数据区
    RX_STATE_SUM            // 已收完数据，等校验字节
} Protocol_RX_State;

/*------------------ 对外接口 ------------------*/
void Protocol_Init(void);              // 初始化串口 + 使能接收中断
void Protocol_RX_Byte(uint8_t dat);    // 串口中断回调中调用：仅入队一个字节
void Protocol_Task(void);              // 主循环调用：取字节跑状态机，解析帧
void Protocol_Send_Status(void);       // 主循环调用：VOFA JustFloat 回传一帧

#endif