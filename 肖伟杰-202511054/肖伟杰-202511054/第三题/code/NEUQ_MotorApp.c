#include "NEUQ_MotorApp.h"
#include "NEUQ_MotorHw.h"
#include "NEUQ_MotorProto.h"
#include "NEUQ_MotorCmd.h"

/* 控制器实例：整个工程只有这一个电机 */
NEUQ_MotorCtrlType g_motor_ctrl;

static NEUQ_MotorProtoType   s_proto;
static NEUQ_MotorCmdType     s_cmd;

static uint8_t  s_fw_buf[NEUQ_MOTOR_FW_MAX_LEN];    /* FireWater 拼帧缓冲 */
static uint32_t s_tick;                             /* 1 ms 节拍计数 */
static uint8_t  s_up_div;                           /* 上传分频计数 */
static volatile uint8_t s_upload_due;               /* ISR 置位、主循环清除 */
static uint32_t s_status_ms;                        /* 上次刷新通信统计的时刻 */

/* ============================== 初始化 ============================== */

void NEUQ_MotorApp_Main(void)
{
    uint32_t now;

    /* 先建控制器（参数取头文件里的默认值），再建硬件 */
    NEUQ_MotorCtrl_Init(&g_motor_ctrl);
    NEUQ_MotorHw_Init();                    /* PWM(PA2) + 方向(PA4) + 编码器(TIM4) + 串口 + TIM5 1 ms */

    now = NEUQ_MotorHw_Millis();
    NEUQ_MotorProto_Init(&s_proto, now);
    NEUQ_MotorCmd_Init(&s_cmd, &g_motor_ctrl, now);

    s_tick       = 0u;
    s_up_div     = 0u;
    s_upload_due = 0u;
    s_status_ms  = now;

    /* 上电状态：不转、等 START 命令（PWM = 0、方向回正转、run = 0） */
    NEUQ_MotorCtrl_Stop(&g_motor_ctrl);
    NEUQ_MotorHw_SetOutput(0.0f, 1u);

    /* **************************** 主循环 **************************** */
    while (1)
    {
        NEUQ_MotorApp_Task();
    }
}

/* 累计编码器计数（只增不减的诊断量；调试器直接看它，或用 ch10 在 VOFA+ 上看）。
   用途：标定 NEUQ_MOTOR_COUNTS_PER_REV —— 断电后用手把输出轴转整整 N 圈，读这个值的增量 Δ，
   则"输出轴一圈计数 = Δ/N"，再按 NEUQ_MotorCtrl.h 的说明换算成 ENC_CPR / GEAR_RATIO。 */
volatile int32_t g_enc_total = 0;


/* ============================== 1 ms 节拍（TIM5 中断里） ============================== */

void NEUQ_MotorApp_Tick(int32_t delta)
{
    /* ① 每 1 ms：把计数增量交给控制器累加（测速窗口就是这几个增量之和） */
    NEUQ_MotorCtrl_FeedTick(&g_motor_ctrl, delta);
    g_enc_total += delta;
    s_tick++;

    /* ② 每 5 ms：完整控制计算 + PWM 更新（200 Hz，定周期，与主循环无关） */
    if ((s_tick % (uint32_t)NEUQ_MOTORH_CTRL_TICKS) == 0u)
    {
        NEUQ_MotorCtrl_ControlTick(&g_motor_ctrl);
        NEUQ_MotorHw_SetOutput(g_motor_ctrl.out.duty, g_motor_ctrl.out.dir_forward);
    }

    /* ③ 每 10 ms：置上传标志，真正的拼帧/发送在主循环 */
    s_up_div++;
    if (s_up_div >= (uint8_t)NEUQ_MOTORH_UPLINK_TICKS)
    {
        s_up_div     = 0u;
        s_upload_due = 1u;
    }
}

/* ============================== 主循环任务 ============================== */

void NEUQ_MotorApp_Task(void)
{
    uint8_t  byte;
    uint32_t now = NEUQ_MotorHw_Millis();

    /* ① 发送泵：TXE 就绪才吐一个字节，永远不等待 */
    NEUQ_MotorHw_TxPump();

    /* ② 半帧保护：一帧凑了 200 ms 还没齐就丢掉重新找帧头 */
    NEUQ_MotorProto_Stall(&s_proto, now);

    /* ③ 解包：把接收队列里的字节全部喂进状态机，有几帧解几帧（粘包/分次到达都能解） */
    while (NEUQ_MotorHw_RxPop(&byte) != 0u)
    {
        NEUQ_MotorProtoFrameType frame;

        if (NEUQ_MotorProto_Feed(&s_proto, byte, now, &frame) != 0u)
        {
            /* 整帧长度凑齐才生效；坏帧在协议层就整帧丢掉了，这里根本看不到 */
            (void)NEUQ_MotorCmd_HandleFrame(&s_cmd, &frame, now);
        }
    }

    /* ④ 到点上传一帧 FireWater 曲线（100 Hz，10 ms；主循环偶尔慢一拍也不会打乱节拍） */
    if (s_upload_due != 0u)
    {
        uint16_t n;

        s_upload_due = 0u;
        n = NEUQ_MotorCmd_BuildFireWater(&s_cmd, s_fw_buf, now);
        s_fw_buf[n] = (uint8_t)'\n';
        n++;
        NEUQ_MotorHw_TxFrame(s_fw_buf, n);
    }

    /* ⑤ 通信统计：把环形队列的溢出次数归到命令层，状态位 bit3 才看得见"到底丢没丢字节" */
    if ((uint32_t)(now - s_status_ms) >= 100u)
    {
        s_status_ms       = now;
        s_cmd.rx_overflow = NEUQ_MotorHw_RxDrop();
        s_cmd.tx_overflow = NEUQ_MotorHw_TxDrop();
    }
}
