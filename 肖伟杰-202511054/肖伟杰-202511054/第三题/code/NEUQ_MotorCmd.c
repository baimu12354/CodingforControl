#include "NEUQ_MotorCmd.h"

/* 累计编码器计数（定义在 NEUQ_MotorApp.c，1 ms 中断里累加）：
 * 只用于 ch10 回传与"每圈计数"标定，不参与任何控制计算。 */
extern volatile int32_t g_enc_total;

/* ============================== 参数校验小工具：先把 NaN/±Inf 挡掉再做范围判断 ============================== */
/* 不用 isnan()/isinf()：ARMCC + microlib 下 math.h 的这两个宏不保证可用，自己判更稳 */
static uint8_t Cmd_FloatOk(float v, float lo, float hi)
{
    if (v != v)                             /* NaN：自己不等于自己 */
    {
        return 0u;
    }
    if (v > 3.0e38f || v < -3.0e38f)        /* ±Inf / 离谱的大数 */
    {
        return 0u;
    }
    if (v < lo || v > hi)
    {
        return 0u;
    }
    return 1u;
}

/* ============================== 初始化 ============================== */

void NEUQ_MotorCmd_Init(NEUQ_MotorCmdType *c, NEUQ_MotorCtrlType *ctrl, uint32_t now_ms)
{
    c->ctrl          = ctrl;
    c->last_seq      = 0u;
    c->last_cmd      = 0u;
    c->last_ack      = NEUQ_MOTOR_ACK_OK;
    c->bad_val       = 0u;
    c->t_last_ok_ms  = now_ms - 1001u;  /* 上电还没收到命令：bit4 先为 0 */
    c->rx_overflow   = 0u;
    c->tx_overflow   = 0u;
}

/* ============================== 命令解包生效 ============================== */

uint8_t NEUQ_MotorCmd_HandleFrame(NEUQ_MotorCmdType *c, const NEUQ_MotorProtoFrameType *f, uint32_t now_ms)
{
    NEUQ_MotorCtrlType *m = c->ctrl;
    uint8_t ack;
    float   v0;
    float   v1;
    float   v2;

    c->last_seq = f->seq;
    c->last_cmd = f->cmd;

    switch (f->cmd)
    {
        /* ---------- 启动闭环：载荷 0（沿用上次目标）或 4（顺带设目标） ---------- */
        case NEUQ_MOTOR_CMD_START:
            if (f->len != 0u && f->len != 4u)
            {
                ack = NEUQ_MOTOR_ACK_BAD_LEN;
                break;
            }
            if (f->len == 4u)
            {
                v0 = NEUQ_MotorProto_GetF32(&f->payload[0]);
                if (Cmd_FloatOk(v0, -NEUQ_MOTOR_RPM_LIMIT, NEUQ_MOTOR_RPM_LIMIT) == 0u)
                {
                    c->bad_val++;
                    ack = NEUQ_MOTOR_ACK_BAD_PARAM;
                    break;
                }
                NEUQ_MotorCtrl_SetTarget(m, v0);
            }
            /* 先清 PID 状态、再对齐目标（顺序反了会让启动第一拍的 D 项踢一脚） */
            NEUQ_MotorCtrl_Start(m);
            ack = NEUQ_MOTOR_ACK_OK;
            break;

        /* ---------- 停止：PWM 归零、测速与 PID 状态清零；目标转速保留 ---------- */
        case NEUQ_MOTOR_CMD_STOP:
            if (f->len != 0u)
            {
                ack = NEUQ_MOTOR_ACK_BAD_LEN;
                break;
            }
            NEUQ_MotorCtrl_Stop(m);         /* 下一拍控制中断里就会把 PA2 写 0 */
            ack = NEUQ_MOTOR_ACK_OK;
            break;

        /* ---------- 目标转速：运行中随时可改 ---------- */
        case NEUQ_MOTOR_CMD_SET_SPEED:
            if (f->len != 4u)
            {
                ack = NEUQ_MOTOR_ACK_BAD_LEN;
                break;
            }
            v0 = NEUQ_MotorProto_GetF32(&f->payload[0]);
            if (Cmd_FloatOk(v0, -NEUQ_MOTOR_RPM_LIMIT, NEUQ_MOTOR_RPM_LIMIT) == 0u)
            {
                c->bad_val++;
                ack = NEUQ_MOTOR_ACK_BAD_PARAM;
                break;
            }
            NEUQ_MotorCtrl_SetTarget(m, v0);
            ack = NEUQ_MOTOR_ACK_OK;
            break;

        /* ---------- 单参数 PID（给滑块用）：Kp / Ki / Kd 各一条，载荷 4 字节 ---------- */
        case NEUQ_MOTOR_CMD_SET_KP:
        case NEUQ_MOTOR_CMD_SET_KI:
        case NEUQ_MOTOR_CMD_SET_KD:
            if (f->len != 4u)
            {
                ack = NEUQ_MOTOR_ACK_BAD_LEN;
                break;
            }
            if (Cmd_FloatOk(v0 = NEUQ_MotorProto_GetF32(&f->payload[0]),
                            0.0f, NEUQ_MOTOR_GAIN_LIMIT) == 0u)
            {
                c->bad_val++;
                ack = NEUQ_MOTOR_ACK_BAD_PARAM;
                break;
            }
            /* 只动目标那一个增益，另外两个保持"实际在用的值" —— 滑块"调哪个是哪个" */
            if (f->cmd == NEUQ_MOTOR_CMD_SET_KP)
            {
                NEUQ_MotorCtrl_SetPid(m, v0, m->Ki, m->Kd);
            }
            else if (f->cmd == NEUQ_MOTOR_CMD_SET_KI)
            {
                NEUQ_MotorCtrl_SetPid(m, m->Kp, v0, m->Kd);
            }
            else
            {
                NEUQ_MotorCtrl_SetPid(m, m->Kp, m->Ki, v0);
            }
            ack = NEUQ_MOTOR_ACK_OK;
            break;

        /* ---------- PID 参数：一次传 Kp/Ki/Kd ---------- */
        case NEUQ_MOTOR_CMD_SET_PID:
            if (f->len != 12u)
            {
                ack = NEUQ_MOTOR_ACK_BAD_LEN;
                break;
            }
            v0 = NEUQ_MotorProto_GetF32(&f->payload[0]);
            v1 = NEUQ_MotorProto_GetF32(&f->payload[4]);
            v2 = NEUQ_MotorProto_GetF32(&f->payload[8]);
            if (Cmd_FloatOk(v0, 0.0f, NEUQ_MOTOR_GAIN_LIMIT) == 0u ||
                Cmd_FloatOk(v1, 0.0f, NEUQ_MOTOR_GAIN_LIMIT) == 0u ||
                Cmd_FloatOk(v2, 0.0f, NEUQ_MOTOR_GAIN_LIMIT) == 0u)
            {
                c->bad_val++;
                ack = NEUQ_MOTOR_ACK_BAD_PARAM;
                break;
            }
            NEUQ_MotorCtrl_SetPid(m, v0, v1, v2);
            ack = NEUQ_MOTOR_ACK_OK;
            break;

        default:
            ack = NEUQ_MOTOR_ACK_UNKNOWN;
            break;
    }

    c->last_ack = ack;
    if (ack == NEUQ_MOTOR_ACK_OK)
    {
        c->t_last_ok_ms = now_ms;           /* 只有合法帧才算"链路活着" */
    }
    return ack;
}

/* ============================== 状态位域 ============================== */

uint16_t NEUQ_MotorCmd_StatusBits(const NEUQ_MotorCmdType *c, uint32_t now_ms)
{
    uint16_t s = 0u;

    if (c->ctrl->run != 0u)
    {
        s |= 0x0001u;                       /* bit0 运行中 */
    }
    if (c->last_ack == NEUQ_MOTOR_ACK_OK)
    {
        s |= 0x0002u;                       /* bit1 上一条命令被接受 */
    }
    if (c->bad_val != 0u || c->rx_overflow != 0u || c->tx_overflow != 0u)
    {
        s |= 0x0008u;                       /* bit3 出现过坏帧 / 接收或发送溢出 */
    }
    if ((uint32_t)(now_ms - c->t_last_ok_ms) < 1000u)
    {
        s |= 0x0010u;                       /* bit4 1 s 内收到过合法帧 */
    }
    /* bit6 保留（上行固定 FireWater 文本，恒 0） */
    if (c->ctrl->out.rpm_meas < 0.0f)
    {
        s |= 0x0080u;                       /* bit7 实测转向为负 */
    }
    s |= (uint16_t)((uint16_t)c->last_ack << 8);    /* bit8..11 最近一次 ACK 码 */
    return s;
}

/* ============================== FireWater 文本上行 ============================== */

/* 定点两位小数：不用 printf/sprintf（microlib 下体积大、还可能被重定向到别处），
   手写一个就够，而且保证输出长度可控（一行最多 60 字节左右）。返回写出的字节数 */
static uint16_t Cmd_AppendFloat2(char *dst, float v)
{
    uint16_t n = 0u;
    uint32_t scaled;
    uint32_t ip;
    uint32_t fp;

    if (v != v)
    {
        v = 0.0f;                           /* 双保险：绝不让 "nan" 这种字符串溜进曲线 */
    }

    if (v < 0.0f)
    {
        dst[n] = '-';
        n++;
        v = -v;
    }
    if (v > 999999.0f)
    {
        v = 999999.0f;
    }

    /* 四舍五入到两位小数后按整数打印，避免浮点打印的精度抖动 */
    scaled = (uint32_t)((v * 100.0f) + 0.5f);
    ip     = scaled / 100u;
    fp     = scaled % 100u;

    /* 整数部分 */
    {
        char     tmp[8];
        uint8_t  k = 0u;

        if (ip == 0u)
        {
            tmp[k] = '0';
            k++;
        }
        while (ip > 0u && k < 8u)
        {
            tmp[k] = (char)('0' + (char)(ip % 10u));
            ip /= 10u;
            k++;
        }
        while (k > 0u)
        {
            k--;
            dst[n] = tmp[k];
            n++;
        }
    }

    dst[n] = '.';
    n++;
    dst[n] = (char)('0' + (char)(fp / 10u));
    n++;
    dst[n] = (char)('0' + (char)(fp % 10u));
    n++;

    return n;
}

static uint16_t Cmd_AppendFloat1(char *dst, float v)
{
    uint16_t n = 0u;
    uint32_t scaled;
    uint32_t ip;
    uint32_t fp;

    if (v != v)
    {
        v = 0.0f;
    }
    if (v < 0.0f)
    {
        dst[n] = '-';
        n++;
        v = -v;
    }
    if (v > 999999.0f)
    {
        v = 999999.0f;
    }

    scaled = (uint32_t)((v * 10.0f) + 0.5f);
    ip     = scaled / 10u;
    fp     = scaled % 10u;

    {
        char     tmp[8];
        uint8_t  k = 0u;

        if (ip == 0u)
        {
            tmp[k] = '0';
            k++;
        }
        while (ip > 0u && k < 8u)
        {
            tmp[k] = (char)('0' + (char)(ip % 10u));
            ip /= 10u;
            k++;
        }
        while (k > 0u)
        {
            k--;
            dst[n] = tmp[k];
            n++;
        }
    }

    dst[n] = '.';
    n++;
    dst[n] = (char)('0' + (char)fp);
    n++;

    return n;
}

static uint16_t Cmd_AppendInt(char *dst, int32_t v)
{
    uint16_t n = 0u;
    uint32_t u;
    char     tmp[12];
    uint8_t  k = 0u;

    if (v < 0)
    {
        dst[n] = '-';
        n++;
        u = (uint32_t)(-v);
    }
    else
    {
        u = (uint32_t)v;
    }

    if (u == 0u)
    {
        tmp[k] = '0';
        k++;
    }
    while (u > 0u && k < 12u)
    {
        tmp[k] = (char)('0' + (char)(u % 10u));
        u /= 10u;
        k++;
    }
    while (k > 0u)
    {
        k--;
        dst[n] = tmp[k];
        n++;
    }
    return n;
}

uint16_t NEUQ_MotorCmd_BuildFireWater(const NEUQ_MotorCmdType *c, uint8_t *buf, uint32_t now_ms)
{
    const NEUQ_MotorCtrlType *m = c->ctrl;
    char    *s = (char *)buf;
    uint16_t n = 0u;

    /* ch0 目标转速 */
    n = (uint16_t)(n + Cmd_AppendFloat2(&s[n], m->target_rpm));
    s[n] = ',';
    n++;
    /* ch1 实测转速（滤波后，PID 用的就是它） */
    n = (uint16_t)(n + Cmd_AppendFloat2(&s[n], m->out.rpm_meas));
    s[n] = ',';
    n++;
    /* ch2 实测转速（未滤波） */
    n = (uint16_t)(n + Cmd_AppendFloat2(&s[n], m->out.rpm_raw));
    s[n] = ',';
    n++;
    /* ch3/ch4/ch5 生效 PID */
    n = (uint16_t)(n + Cmd_AppendFloat2(&s[n], m->Kp));
    s[n] = ',';
    n++;
    n = (uint16_t)(n + Cmd_AppendFloat2(&s[n], m->Ki));
    s[n] = ',';
    n++;
    n = (uint16_t)(n + Cmd_AppendFloat2(&s[n], m->Kd));
    s[n] = ',';
    n++;
    /* ch6 本拍 PWM（带符号） */
    n = (uint16_t)(n + Cmd_AppendFloat1(&s[n], m->out.duty));
    s[n] = ',';
    n++;
    /* ch7 状态位域 */
    n = (uint16_t)(n + Cmd_AppendInt(&s[n], (int32_t)NEUQ_MotorCmd_StatusBits(c, now_ms)));
    s[n] = ',';
    n++;
    /* ch8 最近一条命令回显 = SEQ + CMD×256 */
    n = (uint16_t)(n + Cmd_AppendInt(&s[n], (int32_t)c->last_seq + ((int32_t)c->last_cmd << 8)));
    s[n] = ',';
    n++;
    /* ch9 跟踪误差 */
    n = (uint16_t)(n + Cmd_AppendFloat2(&s[n], m->target_rpm - m->out.rpm_meas));
    s[n] = ',';
    n++;
    /* ch10 累计编码器计数（带符号）：标定 COUNTS_PER_REV 用。
     *      电机断电、用手把输出轴转整整 N 圈，读这个值的增量 Δ，
     *      则"输出轴一圈计数 = Δ/N"。详见 NEUQ_MotorCtrl.h 顶部的测速标定说明。 */
    n = (uint16_t)(n + Cmd_AppendInt(&s[n], (int32_t)g_enc_total));

    return n;                               /* 不带换行，调用者补 '\n' */
}
