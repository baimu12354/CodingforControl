#include "NEUQ_MotorRing.h"

/* ============================== 接收队列 ============================== */

void NEUQ_MotorRxRing_Init(NEUQ_MotorRxRingType *r)
{
    r->head = 0u;
    r->tail = 0u;
    r->drop = 0u;
}

uint8_t NEUQ_MotorRxRing_Push(NEUQ_MotorRxRingType *r, uint8_t byte)
{
    uint16_t next = (uint16_t)((r->head + 1u) & (NEUQ_MOTOR_RX_RING_SIZE - 1u));

    if (next == r->tail)
    {
        r->drop++;                          /* 满：丢字节，不等待 */
        return 0u;
    }
    r->data[r->head] = byte;
    r->head = next;
    return 1u;
}

uint8_t NEUQ_MotorRxRing_Pop(NEUQ_MotorRxRingType *r, uint8_t *byte)
{
    if (r->head == r->tail)
    {
        return 0u;
    }
    *byte = r->data[r->tail];
    r->tail = (uint16_t)((r->tail + 1u) & (NEUQ_MOTOR_RX_RING_SIZE - 1u));
    return 1u;
}

/* ============================== 发送队列 ============================== */

void NEUQ_MotorTxRing_Init(NEUQ_MotorTxRingType *r)
{
    r->head = 0u;
    r->tail = 0u;
    r->drop = 0u;
}

uint8_t NEUQ_MotorTxRing_PushFrame(NEUQ_MotorTxRingType *r, const uint8_t *buf, uint16_t len)
{
    uint16_t free_n;
    uint16_t i;

    if ((len == 0u) || (len >= (uint16_t)NEUQ_MOTOR_TX_RING_SIZE))
    {
        return 0u;
    }

    /* 空队列时 free = SIZE-1；满时 free = 0 */
    free_n = (uint16_t)(((uint16_t)(r->tail - r->head - 1u)) & (NEUQ_MOTOR_TX_RING_SIZE - 1u));
    if (free_n < len)
    {
        r->drop++;                          /* 整帧丢弃，绝不阻塞 */
        return 0u;
    }

    for (i = 0u; i < len; i++)
    {
        r->data[r->head] = buf[i];
        r->head = (uint16_t)((r->head + 1u) & (NEUQ_MOTOR_TX_RING_SIZE - 1u));
    }
    return 1u;
}

uint8_t NEUQ_MotorTxRing_Pop(NEUQ_MotorTxRingType *r, uint8_t *byte)
{
    if (r->head == r->tail)
    {
        return 0u;
    }
    *byte = r->data[r->tail];
    r->tail = (uint16_t)((r->tail + 1u) & (NEUQ_MOTOR_TX_RING_SIZE - 1u));
    return 1u;
}
