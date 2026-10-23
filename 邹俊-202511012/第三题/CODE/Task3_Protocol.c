#include <string.h>
#include "Task3_Protocol.h"

// 每收到一个字节前进一步。等待期间保持状态，不要求一帧一次到齐。
enum { WAIT_HEAD = 0, WAIT_CMD, WAIT_DATA, WAIT_TAIL };

void T3_Parser_init(T3_Parser_Struct *parser)
{
    memset(parser, 0, sizeof(*parser));
}

uint8_t T3_Parser_feed(T3_Parser_Struct *parser, uint8_t byte, T3_Frame_Struct *out)
{
    switch (parser->state) {
    case WAIT_HEAD:
        if (byte == T3_FRAME_HEAD) parser->state = WAIT_CMD;
        break;
    case WAIT_CMD:
        parser->frame.cmd = byte;
        parser->index = 0;
        if (byte == T3_CMD_START || byte == T3_CMD_STOP) {
            parser->frame.len = 0;
            parser->state = WAIT_TAIL;
        }
        else if (byte == T3_CMD_SET_TARGET || byte == T3_CMD_SET_PID) {
            parser->frame.len = (byte == T3_CMD_SET_TARGET) ? 2U : 6U;
            parser->state = WAIT_DATA;
        }
        else {
            // 连续收到 AA 时，最后一个 AA 仍可能是帧头。
            parser->state = (byte == T3_FRAME_HEAD) ? WAIT_CMD : WAIT_HEAD;
        }
        break;
    case WAIT_DATA:
        // 数据本身可以含 AA 或 55，必须按长度接收，不能遇到它们就截断。
        parser->frame.data[parser->index++] = byte;
        if (parser->index == parser->frame.len) parser->state = WAIT_TAIL;
        break;
    case WAIT_TAIL:
        parser->state = WAIT_HEAD;
        if (byte == T3_FRAME_TAIL) {
            *out = parser->frame;
            return 1;                    // 完整帧才允许修改控制参数
        }
        // 帧尾不对则丢弃；若恰好是新帧头，继续等待新命令。
        if (byte == T3_FRAME_HEAD) parser->state = WAIT_CMD;
        break;
    default:
        parser->state = WAIT_HEAD;
        break;
    }
    return 0;
}

uint16_t T3_Frame_get_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

int32_t T3_Frame_get_target(const uint8_t *data)
{
    uint16_t value = T3_Frame_get_u16(data);
    // 将16位补码还原成带符号数，例如 FF 9C 对应 -100 rpm。
    return (value >= 0x8000U) ? (int32_t)value - 65536L : (int32_t)value;
}
