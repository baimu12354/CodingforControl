#include "NEUQ_MotorProto.h"

/* 本版协议不带 CRC：帧长 = 5 + LEN，凑齐即交付。 */

/* ============================== 小工具 ============================== */

float NEUQ_MotorProto_GetF32(const uint8_t *p)
{
    union
    {
        uint32_t u;
        float    f;
    } cvt;

    cvt.u = ((uint32_t)p[0])
          | ((uint32_t)p[1] << 8)
          | ((uint32_t)p[2] << 16)
          | ((uint32_t)p[3] << 24);
    return cvt.f;
}

/* ============================== 解析状态机 ============================== */

void NEUQ_MotorProto_Init(NEUQ_MotorProtoType *p, uint32_t now_ms)
{
    p->idx    = 0u;
    p->need   = 0u;
    p->t_last = now_ms;
}

void NEUQ_MotorProto_Stall(NEUQ_MotorProtoType *p, uint32_t now_ms)
{
    if (p->idx == 0u)
    {
        return;
    }
    if ((uint32_t)(now_ms - p->t_last) > (uint32_t)NEUQ_MOTOR_PROTO_HALF_FRAME_TIMEOUT_MS)
    {
        p->idx  = 0u;                       /* 半帧卡死：丢掉重新找帧头 */
        p->need = 0u;
    }
}

uint8_t NEUQ_MotorProto_Feed(NEUQ_MotorProtoType *p, uint8_t byte, uint32_t now_ms, NEUQ_MotorProtoFrameType *out)
{
    p->t_last = now_ms;                     /* 每个字节都刷新，半帧保护才算"半帧" */

    /* ---- 找帧头：不是 0xAA 或者是 0xAA 但下一个不是 0x55，都当链路垃圾丢掉 ---- */
    if (p->idx == 0u)
    {
        if (byte != (uint8_t)NEUQ_MOTOR_PROTO_SOF0)
        {
            return 0u;
        }
    }
    else if (p->idx == 1u)
    {
        if (byte != (uint8_t)NEUQ_MOTOR_PROTO_SOF1)
        {
            /* 0xAA 后面不是 0x55：这个 0xAA 作废，但当前字节可能是一个新的 0xAA，要重新判 */
            p->idx = 0u;
            p->need = 0u;
            if (byte == (uint8_t)NEUQ_MOTOR_PROTO_SOF0)
            {
                p->buf[0] = byte;
                p->idx = 1u;
            }
            return 0u;
        }
    }

    p->buf[p->idx] = byte;
    p->idx++;

    /* ---- 收满帧头：立刻按 LEN 算出整帧长度 ---- */
    if (p->idx == (uint16_t)NEUQ_MOTOR_PROTO_HEAD_LEN)
    {
        uint8_t len = p->buf[4];

        if (len > (uint8_t)NEUQ_MOTOR_PROTO_MAX_PAYLOAD)
        {
            /* LEN 非法：整帧丢弃，重新找帧头（控制状态不动） */
            p->idx = 0u;
            p->need = 0u;
            return 0u;
        }
        p->need = (uint16_t)(NEUQ_MOTOR_PROTO_HEAD_LEN + len + NEUQ_MOTOR_PROTO_CRC_LEN);
    }

    /* ---- 还没凑齐，继续等（分次到达就停在这里，下次接着凑） ---- */
    if (p->need == 0u || p->idx < p->need)
    {
        return 0u;
    }

    /* ---- 凑齐一整帧：本版无 CRC，长度对上就直接交付（坏帧只能靠 SOF 成对 + LEN 合法拦） ---- */
    p->idx = 0u;
    p->need = 0u;

    out->seq = p->buf[2];
    out->cmd = p->buf[3];
    out->len = p->buf[4];
    {
        uint8_t i;
        for (i = 0u; i < out->len; i++)
        {
            out->payload[i] = p->buf[5u + i];
        }
    }

    return 1u;
}
