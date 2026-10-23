/*********************************************************************************************************************
* 文件名称          NEUQ_MotorRing
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第三题：单生产者/单消费者字节环形队列
*
*   RX：串口中断 Push，主循环 Pop；
*   TX：主循环 PushFrame，主循环 TxPump Pop。
*   头尾各只有一个写者，不用 count、不用临界区；容量为 SIZE-1（留一格区分空/满）。
********************************************************************************************************************/
#ifndef _NEUQ_MOTOR_RING_H_
#define _NEUQ_MOTOR_RING_H_

#include <stdint.h>

#define NEUQ_MOTOR_RX_RING_SIZE   512u     /* 2 的幂 */
#define NEUQ_MOTOR_TX_RING_SIZE   512u

typedef struct
{
    uint8_t  data[NEUQ_MOTOR_RX_RING_SIZE];
    uint16_t head;                          /* 生产者写 */
    uint16_t tail;                          /* 消费者写 */
    uint32_t drop;                          /* 满而丢掉的字节数 */
} NEUQ_MotorRxRingType;

typedef struct
{
    uint8_t  data[NEUQ_MOTOR_TX_RING_SIZE];
    uint16_t head;
    uint16_t tail;
    uint32_t drop;                          /* 满而整帧丢掉的次数 */
} NEUQ_MotorTxRingType;

void    NEUQ_MotorRxRing_Init(NEUQ_MotorRxRingType *r);
uint8_t NEUQ_MotorRxRing_Push(NEUQ_MotorRxRingType *r, uint8_t byte);
uint8_t NEUQ_MotorRxRing_Pop(NEUQ_MotorRxRingType *r, uint8_t *byte);

void    NEUQ_MotorTxRing_Init(NEUQ_MotorTxRingType *r);
uint8_t NEUQ_MotorTxRing_PushFrame(NEUQ_MotorTxRingType *r, const uint8_t *buf, uint16_t len);
uint8_t NEUQ_MotorTxRing_Pop(NEUQ_MotorTxRingType *r, uint8_t *byte);

#endif /* _NEUQ_MOTOR_RING_H_ */
