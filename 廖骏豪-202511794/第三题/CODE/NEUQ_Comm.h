/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_Comm
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
*
* 说明：上位机字节包通信层。
*       · 接收：串口字节中断里只做"入环形缓冲"这一件事，解析放主循环，保证中断极短；
*       · 解析：逐字节状态机，天然支持"一帧分几次到达"和"多帧连续到达"；
*       · 发送：按控制节拍在主循环里组帧上传，不占用中断时间。
*       所有参数都通过 NEUQ_Control 的指令邮箱交给中断应用，通信本身不碰控制变量。
********************************************************************************************************************/
#ifndef _NEUQ_COMM_H_
#define _NEUQ_COMM_H_

#include "stm32f4xx_hal.h"

/* 通信统计（调试用，可在 MDK 的 Watch 窗口直接观察） */
typedef struct
{
    uint32_t rx_ok;         /* 成功解包并生效的帧数       */
    uint32_t rx_bad_crc;    /* 历史字段：本协议不带校验，恒为 0 */
    uint32_t rx_bad_len;    /* 长度非法（> NEUQ_FRAME_MAX_DATA）被丢弃的帧数 */
    uint32_t rx_timeout;    /* 半帧超时重新同步的次数     */
    uint32_t rx_overflow;   /* 接收缓冲溢出丢弃的字节数   */
    uint32_t tx_frames;     /* 回传帧数                   */
} NEUQ_CommStats;

/*  函数简介     通信初始化（初始化串口 + 打开接收中断 + 复位解析状态机）
 *  使用示例     NEUQ_Comm_Init();
 *  备注         内部使用 NEUQ_COMM_UART / NEUQ_COMM_BAUD / NEUQ_COMM_RX_IRQ_PRIO 配置            */
void NEUQ_Comm_Init(void);

/*  函数简介     串口接收中断回调体：把一个字节压入接收环形缓冲
 *  参数说明     dat   收到的字节
 *  备注         由 User/NEUQ_Interrupts.c 的 TS_UART_x_RX_CALLBACK 调用，必须极短        */
void NEUQ_Comm_RxByte(uint8_t dat);

/*  函数简介     通信轮询：把环形缓冲里的字节喂给解析状态机，解包成功后更新参数
 *  备注         在主循环里反复调用；单次最多处理 NEUQ_COMM_POLL_MAX_BYTES 个字节，
 *                不会长时间霸占主循环导致回传被拖慢                                     */
void NEUQ_Comm_Poll(void);

/*  函数简介     按节拍回传一帧（目标转速 / 实际转速 / 当前生效 PID 参数）
 *  备注         在主循环里反复调用；帧格式由 NEUQ_Config.h 的 NEUQ_UPLINK_MODE 决定        */
void NEUQ_Comm_Upload(void);

/*  函数简介     读取通信统计 */
const NEUQ_CommStats *NEUQ_Comm_GetStats(void);

#endif /* _NEUQ_COMM_H_ */
