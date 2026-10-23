#include "NEUQ_lib_include.h"


/* 1ms 系统计数, 由 NEUQ_Interrupts.c 的 PIT_CH1 回调维护(帧内超时用) */
extern uint32_t count;

/* ---------------- 运行状态 ---------------- */
volatile float    Q3_target_rpm = 0.0f;
volatile float    Q3_actual_rpm = 0.0f;
volatile float    Q3_duty       = 0.0f;
volatile uint8_t  Q3_running    = 0;

static IncrementalPID Q3_pid;                 /* 库内增量式 PID(单电机速度环) */
static int32_t    Q3_enc_delta;               /* 最近一周期编码器计数(测速原始量) */
volatile uint8_t  Q3_upload_due = 0;          /* 遥测发送请求(中断置位, 主循环消费) */

/* ---------------- 串口接收 FIFO(RX 中断生产, 主循环消费) ---------------- */
static NEUQ_FIFO_t Q3_rx_fifo;
static uint8_t     Q3_rx_buf[256];

/* ---------------- 命令帧解包状态机 ---------------- */
typedef enum {
    Q3_RX_HDR1 = 0,      /* 等帧头第 1 字节 */
    Q3_RX_HDR2,          /* 等帧头第 2 字节 */
    Q3_RX_CMD,           /* 等命令字 */
    Q3_RX_LEN,           /* 等载荷长度 */
    Q3_RX_PAYLOAD,       /* 收载荷 */
    Q3_RX_CHK            /* 等校验和 */
} Q3_RX_STATE_ENUM;

static Q3_RX_STATE_ENUM Q3_rx_state = Q3_RX_HDR1;
static uint8_t  Q3_rx_cmd;                    /* 当前帧命令字 */
static uint8_t  Q3_rx_len;                    /* 当前帧载荷长度 */
static uint8_t  Q3_rx_idx;                    /* 载荷已收字节数 */
static uint8_t  Q3_rx_payload[Q3_RX_PAYLOAD_MAX];
static uint32_t Q3_rx_last_ms = 0;             /* 最近一次收到字节的时刻(帧内超时) */

#define Q3_RX_FRAME_TIMEOUT_MS  (100)         /* 帧内断流超时 -> 状态机复位 */

/* ---------------- 电机驱动: PWM + 方向(沿用实验室驱动板接法) ---------------- */
static void Q3_Motor_SetDuty(int duty)
{
    if (duty >= 0)                            /* 正转: DIR=Low */
    {
        
        NEUQ_PWM_set_duty(RIGHT_MOTOR_PWM_PIN, duty);
        NEUQ_GPIO_write(RIGHT_MOTOR_DIR_PIN, GPIO_Low);
    }
    else                                      /* 反转: DIR=High */
    {
        
        NEUQ_PWM_set_duty(RIGHT_MOTOR_PWM_PIN, -duty);
        NEUQ_GPIO_write(RIGHT_MOTOR_DIR_PIN, GPIO_High);
    }
}

/* ---------------- 启停行为 ---------------- */
static void Q3_Motor_Start(void)
{
    if (Q3_running) return;                   /* 重复 START 幂等 */
    Ipid_parameters_setting(&Q3_pid, Q3_pid.Kp, Q3_pid.Ki, Q3_pid.Kd);  /* 增益不变, 复位误差历史 */
    Q3_duty = 0.0f;                           /* 输出累积器清零, 干净的阶跃起点 */
    Q3_running = 1;
    // NEUQ_PWM_set_duty(RIGHT_MOTOR_PWM_PIN, 5000);
    // NEUQ_GPIO_write(RIGHT_MOTOR_DIR_PIN, GPIO_Low);
}

static void Q3_Motor_Stop(void)
{
    Q3_running = 0;
    Q3_duty = 0.0f;
    Ipid_parameters_setting(&Q3_pid, Q3_pid.Kp, Q3_pid.Ki, Q3_pid.Kd);  /* 清 PID 历史 */
    Q3_Motor_SetDuty(0);                      /* PWM 归零停机(保持方向脚不动) */
}

/* ---------------- 命令执行(仅完整解包且校验通过后调用) ---------------- */
static void Q3_Exec_Cmd(uint8_t cmd, const uint8_t *pay, uint8_t len)
{
    float f;

    switch (cmd)
    {
        case Q3_CMD_START:                     /* 启动: 上电后等待该命令 */
            Q3_Motor_Start();
            break;

        case Q3_CMD_STOP:                      /* 停止: PID 历史复位, 再启动从零开始 */
            Q3_Motor_Stop();
            break;

        case Q3_CMD_TARGET:                    /* 目标转速: 运行中改=曲线阶跃, 停止态改=下次启动生效 */
            if (len >= 4)
            {
                memcpy(&f, pay, 4);
                if (f >  Q3_TARGET_MAX_ABS) f =  Q3_TARGET_MAX_ABS;
                if (f < -Q3_TARGET_MAX_ABS) f = -Q3_TARGET_MAX_ABS;
                Q3_target_rpm = f;
            }
            break;

        case Q3_CMD_P:                       /* PID 在线整定: 只改增益, 不清误差历史(无扰切换) */
            if (len >= 4)
            {
                memcpy(&Q3_pid.Kp, pay , 4);
            }
            break;
        case Q3_CMD_I:                       /* PID 在线整定: 只改增益, 不清误差历史(无扰切换) */
            if (len >= 4)
            {
                memcpy(&Q3_pid.Ki, pay , 4);
            }
            break;
        case Q3_CMD_D:                       /* PID 在线整定: 只改增益, 不清误差历史(无扰切换) */
            if (len >= 4)
            {
                memcpy(&Q3_pid.Kd, pay , 4);
            }
            break;

        case Q3_CMD_QUERY:                     /* 主动请求一帧遥测 */
            Q3_upload_due = 1;
            break;

        default:                               /* 未知命令: 忽略, 遥测帧内容不变 */
            break;
    }
}

/* ---------------- 初始化(独立固件入口调用) ---------------- */
void Q3_Motor_Init(void)
{
    NEUQ_PWM_init(RIGHT_MOTOR_PWM_PIN, Q3_PWM_FREQ_HZ, 0);   /* PWM 输出 0, 上电电机不转 */
    NEUQ_GPIO_init(RIGHT_MOTOR_DIR_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_Encoder_init(RIGHT_MOTOR_ECODER_PIN, RIGHT_MOTOR_ECODER_PIN_A, RIGHT_MOTOR_ECODER_PIN_B);

    // NEUQ_PWM_set_duty(RIGHT_MOTOR_PWM_PIN, 5000);
    // NEUQ_GPIO_write(RIGHT_MOTOR_DIR_PIN, GPIO_Low);

    Ipid_parameters_setting(&Q3_pid, Q3_PID_KP_DEF, Q3_PID_KI_DEF, Q3_PID_KD_DEF);

    NEUQ_FIFO_init(&Q3_rx_fifo, Q3_rx_buf, sizeof(Q3_rx_buf));

    Q3_target_rpm = 0.0f;
    Q3_actual_rpm = 0.0f;
    Q3_duty = 0.0f;
    Q3_running = 0;                           /* 上电待命, 等待启动命令 */
}

/* ---------------- 固定周期闭环(定时中断内, 每 Q3_CTRL_PERIOD_MS) ---------------- */
void Q3_Motor_Tick(void)
{
    /* 1) 编码器 M 法测速: 读本周期计数并清零(库以 0x7FFF 为中点, 有符号) */
    Q3_enc_delta = NEUQ_Encoder_Read(RIGHT_MOTOR_ECODER_PIN) ;
    NEUQ_Encoder_Clear(RIGHT_MOTOR_ECODER_PIN);

    /* 2) 单位换算: 计数/周期 -> rpm
     *    rpm = counts x (60s/min) / (PPR x T_ctrl[s])
     *        = counts x 60000 / (Q3_ENC_COUNTS_PER_REV x Q3_CTRL_PERIOD_MS) */
    Q3_actual_rpm = (float)Q3_enc_delta
                  * (60.0f / ((float)Q3_ENC_COUNTS_PER_REV * 0.01));

    /* 3) 增量式 PID + 输出累积限幅(增量式的抗积分饱和) */
    if (Q3_running)
    {
        Q3_duty += IncrementalPID_Update(&Q3_pid, Q3_target_rpm, Q3_actual_rpm);
        if (Q3_duty >  10000.0f) Q3_duty =  10000.0f;
        if (Q3_duty < -10000.0f) Q3_duty = -10000.0f;
        Q3_Motor_SetDuty((int)Q3_duty);
    }

    /* 4) 请求主循环上传一帧遥测(中断只置位, 发送在主循环, 不拖长中断) */
    Q3_upload_due = 1;
}

/* ---------------- 串口 RX 中断钩子: 每字节入队 ---------------- */
void Q3_Motor_OnRxByte(uint8_t dat)
{
    NEUQ_FIFO_write_byte(&Q3_rx_fifo, dat);    /* 满/满时丢弃, 坏帧由校验和拒收 */
}

static void Q3_Send_Telemetry(void)
{
    uint8_t buf[Q3_TELEM_BYTES];
    float   f[Q3_TELEM_CH_NUM];
    uint8_t i;

    /* volatile 全局先取快照, 再序列化(小端序与 STM32 内存序一致) */
    f[0] = Q3_target_rpm;                     /* CH1 当前生效目标转速 rpm */
    f[1] = Q3_actual_rpm;                     /* CH2 实际转速 rpm */
    f[2] = Q3_pid.Kp;                         /* CH3 当前生效 Kp */
    f[3] = Q3_pid.Ki;                         /* CH4 当前生效 Ki */
    f[4] = Q3_pid.Kd;                         /* CH5 当前生效 Kd */
    f[5] = fabs(Q3_duty);                           /* CH6 PWM 输出 */
    f[6] = Q3_running ? 1.0f : 0.0f;          /* CH7 运行状态 */

    for (i = 0; i < Q3_TELEM_CH_NUM; i++)
        memcpy(buf + 4 * i, &f[i], 4);

    buf[28] = 0x00;                           
    buf[29] = 0x00;
    buf[30] = 0x80;
    buf[31] = 0x7F;

    NEUQ_Uart_write_data(UART_0, buf, Q3_TELEM_BYTES);
    // printf("%d,%d,%d,%d,%d,%d,%d\n",Q3_actual_rpm,Q3_actual_rpm,Q3_pid.Kp,Q3_pid.Ki,Q3_pid.Kd,Q3_duty,Q3-);

}

/* ---------------- 主循环: 解包命令帧 + 遥测上传 ---------------- */
void Q3_Rx_Poll(void)
{
    uint8_t d;

    /* 1) 消费 FIFO 中所有已到字节(状态机跨调用保持 -> 分次到达的一帧自然拼装) */
    while (NEUQ_FIFO_read_byte(&Q3_rx_fifo, &d))
    {
        Q3_rx_last_ms = count;

        switch (Q3_rx_state)
        {
            case Q3_RX_HDR1:
                if (d == Q3_FRAME_HDR1) Q3_rx_state = Q3_RX_HDR2;
                break;

            case Q3_RX_HDR2:
                if (d == Q3_FRAME_HDR2) Q3_rx_state = Q3_RX_CMD;
                else if (d == Q3_FRAME_HDR1) Q3_rx_state = Q3_RX_HDR2;  /* A5 A5 5A 连续帧头 */
                else Q3_rx_state = Q3_RX_HDR1;
                break;

            case Q3_RX_CMD:
                Q3_rx_cmd = d;
                Q3_rx_state = Q3_RX_LEN;
                break;

            case Q3_RX_LEN:
                if (d > Q3_RX_PAYLOAD_MAX)    /* 非法长度: 拒帧并尝试重新同步 */
                {
                    Q3_rx_state = (d == Q3_FRAME_HDR1) ? Q3_RX_HDR2 : Q3_RX_HDR1;
                }
                else
                {
                    Q3_rx_len = d;
                    Q3_rx_idx = 0;
                    Q3_rx_state = (Q3_rx_len > 0) ? Q3_RX_PAYLOAD : Q3_RX_CHK;
                }
                break;

            case Q3_RX_PAYLOAD:
                Q3_rx_payload[Q3_rx_idx++] = d;
                if (Q3_rx_idx >= Q3_rx_len) Q3_rx_state = Q3_RX_CHK;
                break;

            case Q3_RX_CHK:
            {
                uint8_t sum = Q3_rx_cmd + Q3_rx_len;
                uint8_t i;
                for (i = 0; i < Q3_rx_len; i++) sum += Q3_rx_payload[i];

                if (sum == d || d == Q3_CHK_BYPASS)                  /* 校验通过: 完整解包后才更新参数 */
                    Q3_Exec_Cmd(Q3_rx_cmd, Q3_rx_payload, Q3_rx_len);
                /* 校验失败: 丢帧, 状态机回帧头等下一帧 */

                Q3_rx_state = (d == Q3_FRAME_HDR1) ? Q3_RX_HDR2 : Q3_RX_HDR1;
                break;
            }

            default:
                Q3_rx_state = Q3_RX_HDR1;
                break;
        }
    }

    /* 2) 帧内断流超时: 半截帧占用状态机超过 100ms 则复位, 保证后续帧仍可解出 */
    if (Q3_rx_state != Q3_RX_HDR1 &&
        (count - Q3_rx_last_ms) > Q3_RX_FRAME_TIMEOUT_MS)
    {
        Q3_rx_state = Q3_RX_HDR1;
    }

    /* 3) 遥测上传: 中断按控制周期置位, 主循环发送(100Hz, 持续绘制曲线) */
    if (Q3_upload_due)
    {
        Q3_upload_due = 0;
        Q3_Send_Telemetry();
    }
}
