/*
 * NEUQ_Wave.h
 * 把当前选中轴的原始波形和滤波波形成对上传给上位机
 *
 * 一帧里只有两个数，而且这两个数来自同一次采样，单位和纵轴尺度完全一样，
 * 所以在VOFA+里可以直接叠在一起对比。
 * 每次发送都从g_imu里取同一次采样的raw和filtered，中间没有时间错位。
 */
#ifndef _NEUQ_WAVE_H_
#define _NEUQ_WAVE_H_

#include "NEUQ_lib_include.h"
#include "NEUQ_Setting.h"

/*
 * 上传串口，改这一个宏就能换口
 *
 * 当前用UART_0 = USART1 = PA9(TX) / PA10(RX)，115200，也就是printf用的那个口。
 *
 * 既然波形和调试文字共用一个串口，就要注意两件事：
 *   1) NEUQ_MENU.c里的按键调试打印会被自动关掉，不然每按一次键就有一行文字
 *      插进数据流里，上位机收到的就是乱码；
 *   2) 上电自检信息走NEUQ_Wave_Log()，只在开机那一下出现，之后串口上就全是干净数据。
 *
 * 板上其它串口，换接线或者换板子的时候参考：
 *   UART_1 = USART2，PA2=TX / PA3=RX
 *            本板PA2经R5 100Ω接到了电机信号&编码器座，所以端口探针故意不去动它
 *   UART_2 = USART3，PB10/PB11
 *   UART_3 = UART4，PC10=TX / PC11=RX
 *            本板右下角那个三针"串口"排针就是它，针1=GND 针2=PC10 针3=PC11。
 *            用排针的话，针2只能接适配器的RXD；接到适配器的TXD就是两个输出脚对打。
 */
#define NEUQ_WAVE_UART          (UART_0)
#define NEUQ_WAVE_BAUD          (UART_BAUD_115200)

/*
 * 上传协议，要和上位机里选的数据格式一致
 *   FIREWATER：文本，格式 "原始值,滤波值\n"，人能直接看，用普通串口助手也能调试
 *   JUSTFLOAT：二进制，两个float加帧尾 00 00 80 7F，同样速率下省带宽
 */
#define NEUQ_WAVE_PROTO_FIREWATER   (0)
#define NEUQ_WAVE_PROTO_JUSTFLOAT   (1)
#define NEUQ_WAVE_PROTOCOL          (NEUQ_WAVE_PROTO_FIREWATER)

/*
 * 发送超时保护
 * 库函数NEUQ_Uart_write_byte()是 while(!(USARTx->SR & USART_SR_TXE)); 没有超时，
 * 只要USART不置TXE，主循环就永远停在那儿，屏幕定格、按键全废，拔线也不恢复。
 * 这里不用库函数，每个字节最多等NEUQ_WAVE_TX_TIMEOUT_US微秒，等不到就丢掉这一帧，
 * 连续失败NEUQ_WAVE_TX_FAIL_LIMIT帧就自动把上传关掉。
 * 115200下一个字节才87us，2000us的超时非常宽松，正常不会触发。
 */
#define NEUQ_WAVE_TX_TIMEOUT_US     (2000U)     // 单个字节等TXE的超时时间，微秒
#define NEUQ_WAVE_TX_FAIL_LIMIT     (5U)        // 连续失败多少帧就自动停发

void NEUQ_Wave_Init(void);
void NEUQ_Wave_Tick(void);
void NEUQ_Wave_ResetPhase(void);

// 取当前选中轴的波形值，filtered=0取原始1取滤波
float NEUQ_Wave_GetValue(uint8_t axis, uint8_t filtered);

// 本次上电以来真正发成功的帧数
// 排查收不到波形的时候先看这个：数在涨说明单片机确实发出去了，
// 数不动说明单片机这边就没发（Send是OFF，或者超时被自动停了）
uint32_t NEUQ_Wave_GetFrames(void);

// 最近一次发送正不正常，0表示等TXE超时、上传已被自动关闭
uint8_t NEUQ_Wave_IsUartOk(void);

// 上电自检，发一帧 "0.0,0.0\n"，返回1正常0异常
uint8_t NEUQ_Wave_SelfTest(void);

// 上电时往PA9和PC10各发一行带名字的测试文字，用来确认线接在哪个脚上
void NEUQ_Wave_PortProbe(void);

// 往上传串口打一行文字，自带超时，不会卡死主循环
void NEUQ_Wave_Log(const char *text);

#endif /* _NEUQ_WAVE_H_ */
