/* task_motor.c - 第三题任务入口 */

#include "task_motor.h"
#include "board_config.h"
#include "host_link.h"
#include "motor.h"
#include "vofa.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

static volatile uint32_t s_control_tick  = 0;   /* 控制节拍，中断里累加 */
static uint32_t          s_upload_tick   = 0;   /* 主循环已回传到第几拍 */
static uint32_t          s_cmd_ok        = 0;   /* 被接受的指令条数 */
static uint32_t          s_cmd_rejected  = 0;   /* 被拒绝的指令条数 */

void Task_Motor_Init(void)
{
    NEUQ_Delay_init();

    Motor_Init();
    HostLink_Init();

    s_control_tick = 0;
    s_upload_tick  = 0;

    NEUQ_PIT_init_ms(PIT_CH2, T3_CONTROL_PERIOD_MS);    /* TIM7 启动定周期控制 */
}

void Task_Motor_ControlTick(void)
{
    s_control_tick++;
    Motor_ControlIsr();
}

/* 执行启停/目标/PID 指令 */
static void Task_Motor_HandleFrame(const host_frame_t *frame)
{
    switch (frame->cmd)
    {
        case T3_CMD_START:
            Motor_Start();
            s_cmd_ok++;
            break;

        case T3_CMD_STOP:
            Motor_Stop();
            s_cmd_ok++;
            break;

        case T3_CMD_SET_TARGET:
            if (frame->len == 4U)
            {
                Motor_SetTargetRpm(HostLink_PayloadToFloat(frame->payload));
                s_cmd_ok++;
            }
            else
            {
                s_cmd_rejected++;
            }
            break;

        case T3_CMD_SET_TARGET_SLIDER:
            /* VOFA+ 滑动条发的帧：A5 13 04 <float 小端> 5A，不带 CRC。
             * 每个滑块数值都是一次事件，拖动时会连续下发，正好实现实时调速。 */
            if (frame->len == 4U)
            {
                Motor_SetTargetRpm(HostLink_PayloadToFloat(frame->payload));
                s_cmd_ok++;
            }
            else
            {
                s_cmd_rejected++;
            }
            break;

        /* PID 仅允许在停止状态修改 */
        case T3_CMD_SET_PID:
            if (frame->len != 12U)
            {
                s_cmd_rejected++;
            }
            else if (Motor_IsEnabled() != 0U)
            {
                s_cmd_rejected++;
            }
            else
            {
                Motor_SetPid(HostLink_PayloadToFloat(&frame->payload[0]),
                             HostLink_PayloadToFloat(&frame->payload[4]),
                             HostLink_PayloadToFloat(&frame->payload[8]));
                s_cmd_ok++;
            }
            break;

        case T3_CMD_QUERY:
            /* 查询：把回传节拍对齐到当前，让主循环立刻补发一帧 */
            s_upload_tick = (uint32_t)(s_control_tick - T3_UPLOAD_TICKS);
            s_cmd_ok++;
            break;

        default:
            s_cmd_rejected++;
            break;
    }
}

/* 20 Hz 回传 10 通道 */
static void Task_Motor_Upload(void)
{
    motor_snapshot_t snap;
    float            ch[10];

    Motor_GetSnapshot(&snap);

    ch[0] = snap.target_rpm;
    ch[1] = snap.actual_rpm;
    ch[2] = snap.kp;
    ch[3] = snap.ki;
    ch[4] = snap.kd;
    ch[5] = (snap.enabled != 0U) ? 1.0f : 0.0f;
    ch[6] = (float)snap.pwm_out;
    ch[7] = (float)snap.counts_last;
    ch[8] = (float)s_cmd_ok;
    ch[9] = (float)s_cmd_rejected;

    Vofa_SendFloats(ch, 10U);
}

void Task_Motor_Loop(void)
{
    host_frame_t frame;

    /* 1. 把 FIFO 里已经到达的字节全部解析完（可能一次到多帧） */
    while (HostLink_Poll(&frame) != 0U)
    {
        Task_Motor_HandleFrame(&frame);
    }

    /* 2. 每 5 个控制周期回传一帧，即 20 Hz */
    if ((uint32_t)(s_control_tick - s_upload_tick) >= T3_UPLOAD_TICKS)
    {
        s_upload_tick = s_control_tick;
        Task_Motor_Upload();
    }
}
