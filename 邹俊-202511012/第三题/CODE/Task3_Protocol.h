/* 第三题：短字节包。串口助手选择 HEX 发送，不附加回车换行。
 * 格式：AA CMD DATA 55；数据长度由 CMD 决定，不需要长度字节和校验和。
 * 01 启动：AA 01 55       02 停止：AA 02 55
 * 03 目标：AA 03 HH LL 55，带符号16位整数 rpm，高字节在前。
 * 04 PID ：AA 04 PH PL IH IL DH DL 55，每个参数为无符号16位整数 / 100。
 * 示例：100 rpm = AA 03 00 64 55；PID 2/0.12/0 = AA 04 00 C8 00 0C 00 00 55。
 * 只在收齐规定长度且帧尾正确后交给电机模块；支持分次收帧和连续多帧。
 * 无校验和不能检测数据区的所有传输错误；回传值用于确认实际生效参数。
 */
#ifndef _TASK3_PROTOCOL_H_
#define _TASK3_PROTOCOL_H_
#include <stdint.h>

#define T3_FRAME_HEAD       (0xAAU)
#define T3_FRAME_TAIL       (0x55U)
#define T3_FRAME_MAX_DATA   (6U)
#define T3_CMD_START        (0x01U)
#define T3_CMD_STOP         (0x02U)
#define T3_CMD_SET_TARGET   (0x03U)
#define T3_CMD_SET_PID      (0x04U)

typedef struct {
    uint8_t cmd;
    uint8_t len;             // 解析器根据命令填入，串口上不发送此字段
    uint8_t data[T3_FRAME_MAX_DATA];
} T3_Frame_Struct;

typedef struct {
    uint8_t state;
    uint8_t index;
    T3_Frame_Struct frame;
} T3_Parser_Struct;

void T3_Parser_init(T3_Parser_Struct *parser);
uint8_t T3_Parser_feed(T3_Parser_Struct *parser, uint8_t byte, T3_Frame_Struct *out);
uint16_t T3_Frame_get_u16(const uint8_t *data);
int32_t T3_Frame_get_target(const uint8_t *data);
#endif
