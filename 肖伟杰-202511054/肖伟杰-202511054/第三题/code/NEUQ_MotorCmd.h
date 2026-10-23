/*********************************************************************************************************************
* 文件名称          NEUQ_MotorCmd
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 适用平台          STM32F407VE
*
* 功能              第三题：命令解包生效 + FireWater 文本回传
*
* 上行（每 10 ms 一行，11 个通道，逗号分隔、\n 结尾）：
*   目标,实测,原始,Kp,Ki,Kd,PWM,状态,回显,误差,累计计数
*
*   回传值全部取自 g_motor_ctrl 里"实际正在用"的字段，所以上位机看到的就是控制环
*   真正使用的目标 / PID / 转速。
********************************************************************************************************************/
#ifndef _NEUQ_MOTOR_CMD_H_
#define _NEUQ_MOTOR_CMD_H_

#include <stdint.h>
#include "NEUQ_MotorProto.h"
#include "NEUQ_MotorCtrl.h"

/* ACK 码 */
#define NEUQ_MOTOR_ACK_OK            0u
#define NEUQ_MOTOR_ACK_BAD_PARAM     2u    /* NaN / ±Inf / 越界 */
#define NEUQ_MOTOR_ACK_BAD_LEN       3u    /* 载荷长度不符 */
#define NEUQ_MOTOR_ACK_UNKNOWN       5u    /* 未知命令码 */

#define NEUQ_MOTOR_FW_CHANNELS       11u
#define NEUQ_MOTOR_FW_MAX_LEN        80u

typedef struct
{
    NEUQ_MotorCtrlType *ctrl;
    uint8_t  last_seq;
    uint8_t  last_cmd;
    uint8_t  last_ack;
    uint32_t bad_val;                       /* 参数非法次数（状态位 bit3） */
    uint32_t t_last_ok_ms;                  /* 最近一条合法命令时刻（链路活性） */
    uint32_t rx_overflow;
    uint32_t tx_overflow;
} NEUQ_MotorCmdType;

void     NEUQ_MotorCmd_Init(NEUQ_MotorCmdType *c, NEUQ_MotorCtrlType *ctrl, uint32_t now_ms);
uint8_t  NEUQ_MotorCmd_HandleFrame(NEUQ_MotorCmdType *c, const NEUQ_MotorProtoFrameType *f, uint32_t now_ms);
uint16_t NEUQ_MotorCmd_BuildFireWater(const NEUQ_MotorCmdType *c, uint8_t *buf, uint32_t now_ms);
uint16_t NEUQ_MotorCmd_StatusBits(const NEUQ_MotorCmdType *c, uint32_t now_ms);

#endif /* _NEUQ_MOTOR_CMD_H_ */
