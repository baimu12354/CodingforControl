#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

/* ==================== UART 中断 -> 主循环环形缓冲区 ==================== */
static volatile uint8_t s_rx_buffer[MOTOR_RX_BUFFER_SIZE];
static volatile uint8_t s_rx_write = 0U;
static volatile uint8_t s_rx_read = 0U;
static volatile uint32_t s_rx_overflow = 0UL;

/* ==================== 返回给 VOFA 的命令状态 ==================== */
static uint8_t s_last_command = MOTOR_CMD_QUERY;
static uint8_t s_last_result = MOTOR_RESULT_OK;
static uint8_t s_status_pending = 1U;

/* ==================== float 与 4 个小端字节互转 ==================== */
static float Protocol_ReadFloatLE(const uint8_t *data)
{
    union
    {
        float value;
        uint8_t bytes[4];
    } converter;
    uint8_t i;

    for (i = 0U; i < 4U; i++) converter.bytes[i] = data[i];
    return converter.value;
}

static void Protocol_WriteFloatLE(uint8_t *data, float value)
{
    union
    {
        float value;
        uint8_t bytes[4];
    } converter;
    uint8_t i;

    converter.value = value;
    for (i = 0U; i < 4U; i++) data[i] = converter.bytes[i];
}

static float Protocol_AbsFloat(float value)
{
    return value >= 0.0f ? value : -value;
}

static uint8_t Protocol_ValidFinite(float value)
{
    /* NaN 是唯一一个与自身不相等的浮点数；范围判断同时排除无穷大。 */
    return (uint8_t)((value == value) &&
                     value > -100000.0f &&
                     value < 100000.0f);
}

static void Protocol_SaveResult(uint8_t command, uint8_t result)
{
    s_last_command = command;
    s_last_result = result;
    s_status_pending = 1U;
}

/* ==================== 完整帧的命令执行 ==================== */
static uint8_t Protocol_SetSinglePid(uint8_t parameter, float value)
{
    Motor_Control_Status status;

    if (!Protocol_ValidFinite(value) || value < 0.0f)
    {
        return MOTOR_RESULT_BAD_VALUE;
    }

    Motor_Control_GetStatus(&status);
    if (parameter == MOTOR_PID_PARAMETER_KP)
    {
        status.kp = value;
    }
    else if (parameter == MOTOR_PID_PARAMETER_KI)
    {
        status.ki = value;
    }
    else if (parameter == MOTOR_PID_PARAMETER_KD)
    {
        status.kd = value;
    }
    else
    {
        return MOTOR_RESULT_BAD_PARAMETER;
    }

    Motor_Control_SetPid(status.kp, status.ki, status.kd);
    return MOTOR_RESULT_OK;
}

static uint8_t Protocol_SetAllPid(const uint8_t *payload)
{
    float kp = Protocol_ReadFloatLE(&payload[0]);
    float ki = Protocol_ReadFloatLE(&payload[4]);
    float kd = Protocol_ReadFloatLE(&payload[8]);

    if (!Protocol_ValidFinite(kp) ||
        !Protocol_ValidFinite(ki) ||
        !Protocol_ValidFinite(kd) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f)
    {
        return MOTOR_RESULT_BAD_VALUE;
    }

    Motor_Control_SetPid(kp, ki, kd);
    return MOTOR_RESULT_OK;
}

static void Protocol_ExecuteCommand(uint8_t command,
                                    const uint8_t *payload,
                                    uint8_t length)
{
    float value;
    uint8_t result = MOTOR_RESULT_OK;

    switch (command)
    {
        case MOTOR_CMD_START:
            if (length != 0U) result = MOTOR_RESULT_BAD_LENGTH;
            else if (Motor_Control_HasFault() != 0U)
                result = MOTOR_RESULT_CONTROL_FAULT;
            else Motor_Control_SetRunning(1U);
            break;

        case MOTOR_CMD_STOP:
            if (length != 0U) result = MOTOR_RESULT_BAD_LENGTH;
            else
            {
                Motor_Control_SetRunning(0U);
                /* STOP 同时作为故障确认；排除接线问题后可再次 START。 */
                Motor_Control_ClearFault();
                /* STOP 后 I10 从 0 开始，便于重新累计编码器计数。 */
                Motor_Control_ResetEncoderTotal();
            }
            break;

        case MOTOR_CMD_SET_SPEED:
            if (length != 4U)
            {
                result = MOTOR_RESULT_BAD_LENGTH;
            }
            else
            {
                value = Protocol_ReadFloatLE(payload);
                if (!Protocol_ValidFinite(value) ||
                    Protocol_AbsFloat(value) > MOTOR_MAX_TARGET_RPM)
                {
                    result = MOTOR_RESULT_BAD_VALUE;
                }
                else
                {
                    /* 题目要求运行中也能改变目标速度。 */
                    Motor_Control_SetTargetRpm(value);
                }
            }
            break;

        case MOTOR_CMD_SET_PID:
            if (length != 5U && length != 12U)
            {
                result = MOTOR_RESULT_BAD_LENGTH;
            }
            else if (Motor_Control_IsRunning() != 0U)
            {
                /* 题目要求 PID 只允许在停机状态修改。 */
                result = MOTOR_RESULT_BUSY;
            }
            else if (length == 5U)
            {
                value = Protocol_ReadFloatLE(&payload[1]);
                result = Protocol_SetSinglePid(payload[0], value);
            }
            else
            {
                result = Protocol_SetAllPid(payload);
            }
            break;

        case MOTOR_CMD_QUERY:
            if (length != 0U) result = MOTOR_RESULT_BAD_LENGTH;
            break;

        default:
            result = MOTOR_RESULT_UNKNOWN_CMD;
            break;
    }

    Protocol_SaveResult(command, result);
}

/* ==================== 可跨多次接收、可连续解析多帧的状态机 ==================== */
static void Protocol_ParseByte(uint8_t byte)
{
    static uint8_t state = MOTOR_PARSE_WAIT_HEAD_1;
    static uint8_t command = 0U;
    static uint8_t length = 0U;
    static uint8_t index = 0U;
    static uint8_t payload[MOTOR_MAX_PAYLOAD_SIZE];
    static uint32_t last_byte_ms = 0UL;
    uint32_t now_ms = HAL_GetTick();

    if (state != MOTOR_PARSE_WAIT_HEAD_1 &&
        (uint32_t)(now_ms - last_byte_ms) > MOTOR_FRAME_TIMEOUT_MS)
    {
        state = MOTOR_PARSE_WAIT_HEAD_1;
    }
    last_byte_ms = now_ms;

    switch (state)
    {
        case MOTOR_PARSE_WAIT_HEAD_1:
            if (byte == MOTOR_FRAME_HEAD_1)
                state = MOTOR_PARSE_WAIT_HEAD_2;
            break;

        case MOTOR_PARSE_WAIT_HEAD_2:
            if (byte == MOTOR_FRAME_HEAD_2)
                state = MOTOR_PARSE_WAIT_COMMAND;
            else
                state = (byte == MOTOR_FRAME_HEAD_1) ?
                        MOTOR_PARSE_WAIT_HEAD_2 : MOTOR_PARSE_WAIT_HEAD_1;
            break;

        case MOTOR_PARSE_WAIT_COMMAND:
            command = byte;
            state = MOTOR_PARSE_WAIT_LENGTH;
            break;

        case MOTOR_PARSE_WAIT_LENGTH:
            length = byte;
            index = 0U;
            if (length > MOTOR_MAX_PAYLOAD_SIZE)
            {
                Protocol_SaveResult(command, MOTOR_RESULT_BAD_LENGTH);
                state = MOTOR_PARSE_WAIT_HEAD_1;
            }
            else if (length == 0U)
            {
                state = MOTOR_PARSE_WAIT_TAIL_1;
            }
            else
            {
                state = MOTOR_PARSE_WAIT_PAYLOAD;
            }
            break;

        case MOTOR_PARSE_WAIT_PAYLOAD:
            payload[index++] = byte;
            if (index >= length) state = MOTOR_PARSE_WAIT_TAIL_1;
            break;

        case MOTOR_PARSE_WAIT_TAIL_1:
            if (byte == MOTOR_FRAME_TAIL_1)
                state = MOTOR_PARSE_WAIT_TAIL_2;
            else
                state = (byte == MOTOR_FRAME_HEAD_1) ?
                        MOTOR_PARSE_WAIT_HEAD_2 : MOTOR_PARSE_WAIT_HEAD_1;
            break;

        default: /* MOTOR_PARSE_WAIT_TAIL_2 */
            if (byte == MOTOR_FRAME_TAIL_2)
                Protocol_ExecuteCommand(command, payload, length);
            state = (byte == MOTOR_FRAME_HEAD_1) ?
                    MOTOR_PARSE_WAIT_HEAD_2 : MOTOR_PARSE_WAIT_HEAD_1;
            break;
    }
}

/* ==================== 对外接口 ==================== */
void Motor_Protocol_Init(void)
{
    s_rx_write = 0U;
    s_rx_read = 0U;
    s_rx_overflow = 0UL;
    s_last_command = MOTOR_CMD_QUERY;
    s_last_result = MOTOR_RESULT_OK;
    s_status_pending = 1U;

    /* 第三题只用这一个串口：同一串口收命令并发送 JustFloat 数据。 */
    NEUQ_Uart_init(MOTOR_UART, MOTOR_UART_BAUD);
    NEUQ_Uart_Enable_RX_Interrupt(MOTOR_UART,
                                  MOTOR_UART_INTERRUPT_PRIORITY);
}

void Motor_Protocol_RxByte(uint8_t data)
{
    uint8_t next = (uint8_t)((s_rx_write + 1U) &
                             (MOTOR_RX_BUFFER_SIZE - 1U));

    if (next != s_rx_read)
    {
        s_rx_buffer[s_rx_write] = data;
        s_rx_write = next;
    }
    else
    {
        s_rx_overflow++;
    }
}

void Motor_Protocol_ProcessRx(void)
{
    uint8_t data;
    uint8_t processed = 0U;

    /* 每轮限制解析量，防止连续串口数据长期占住主循环。 */
    while (s_rx_read != s_rx_write &&
           processed < MOTOR_MAX_RX_BYTES_PER_LOOP)
    {
        data = s_rx_buffer[s_rx_read];
        s_rx_read = (uint8_t)((s_rx_read + 1U) &
                              (MOTOR_RX_BUFFER_SIZE - 1U));
        Protocol_ParseByte(data);
        processed++;
    }
}

uint8_t Motor_Protocol_StatusPending(void)
{
    return s_status_pending;
}

void Motor_Protocol_SendVofaStatus(void)
{
    Motor_Control_Status status;
    uint8_t frame[MOTOR_VOFA_CHANNEL_COUNT * 4U + 4U];
    uint32_t tail = MOTOR_VOFA_CHANNEL_COUNT * 4U;

    Motor_Control_GetStatus(&status);

    /* 通道名和含义固定，不能改动。 */
    Protocol_WriteFloatLE(&frame[0],  status.target_rpm);          /* I0 target_rpm */
    Protocol_WriteFloatLE(&frame[4],  status.actual_rpm);          /* I1 actual_rpm */
    Protocol_WriteFloatLE(&frame[8],  status.kp);                  /* I2 kp */
    Protocol_WriteFloatLE(&frame[12], status.ki);                  /* I3 ki */
    Protocol_WriteFloatLE(&frame[16], status.kd);                  /* I4 kd */
    Protocol_WriteFloatLE(&frame[20], (float)status.running);      /* I5 running */
    Protocol_WriteFloatLE(&frame[24], (float)s_last_command);      /* I6 last command */
    Protocol_WriteFloatLE(&frame[28], (float)s_last_result);       /* I7 last result */
    Protocol_WriteFloatLE(&frame[32], (float)s_rx_overflow);       /* I8 RX overflow */
    Protocol_WriteFloatLE(&frame[36], (float)status.fault);        /* I9 motor fault */
    Protocol_WriteFloatLE(&frame[40],
                          (float)status.encoder_total_count);      /* I10 signed total count */

    /* VOFA+ JustFloat 帧尾：00 00 80 7F。 */
    frame[tail + 0U] = 0x00U;
    frame[tail + 1U] = 0x00U;
    frame[tail + 2U] = 0x80U;
    frame[tail + 3U] = 0x7FU;

    s_status_pending = 0U;
    NEUQ_Uart_write_data(MOTOR_UART, frame, sizeof(frame));
}
