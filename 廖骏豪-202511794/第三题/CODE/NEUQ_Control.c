/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_Control
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
********************************************************************************************************************/
#include "NEUQ_Config.h"
#include "NEUQ_Control.h"
#include "NEUQ_Speed.h"
#include "NEUQ_Motor.h"
#include "NEUQ_SpeedPID.h"

/*===================================================================================================================
 * 指令邮箱
 *   写：主循环（NEUQ_Control_ReqXxx）
 *   读：控制中断（NEUQ_Control_ISR）
 *   同步：主循环全部字段写完后再把 seq 加一；中断发现 seq 变化才整组取用。
 *         主循环不会在中断执行期间运行，因此不会出现"取到半套参数"的情况；
 *         若两条命令在一个控制周期内先后到达，中断取到的是最后一次的完整状态，不会丢参数。
 *=================================================================================================================*/
typedef struct
{
    volatile uint32_t seq;              /* 写序号，变化即代表有新状态           */
    volatile float    target;           /* 期望目标转速(RPM)                    */
    volatile float    kp;               /* 期望 Kp                              */
    volatile float    ki;               /* 期望 Ki                              */
    volatile float    kd;               /* 期望 Kd                              */
    volatile float    kff;              /* 期望前馈增益                         */
    volatile float    slew;             /* 期望输出变化率限幅                   */
    volatile float    openloop_duty;    /* 开环调试占空比                       */
    volatile uint8_t  enable;           /* 期望运行状态 1=运行 0=停止           */
    volatile uint8_t  openloop;         /* 期望开环模式 1=开环调试              */
    volatile uint8_t  pid_dirty;        /* 1 = PID 参数有更新待应用             */
    volatile uint8_t  clear_req;        /* 1 = 请求清 PID 状态                  */
    volatile uint8_t  pid_sel;          /* 单参数 PID：1=Kp 2=Ki 3=Kd           */
    volatile float    pid_val;          /* 单参数 PID：该参数的新值             */
    volatile uint8_t  pid_sel_dirty;    /* 1 = 单参数 PID 有更新待应用           */
} NEUQ_CmdBox;

static NEUQ_CmdBox   s_box;               /* 指令邮箱                             */
static uint32_t    s_box_applied = 0xFFFFFFFFu;  /* 中断已应用的序号            */

/* ---------------- 中断私有状态 ---------------- */
static NEUQ_Pid_t    s_pid;               /* 速度环调节器                         */
static NEUQ_Snapshot s_snap;              /* 回传快照（中断写、主循环读）         */
static uint32_t    s_tick = 0;          /* 控制节拍计数                         */
static float       s_target = 0.0f;     /* 目标转速(RPM)                        */
static float       s_ref = 0.0f;        /* 当前生效参考值（开斜坡时为斜坡输出） */
static uint8_t     s_enable = 0;        /* 运行标志                             */
static uint8_t     s_openloop = 0;      /* 开环调试标志                         */
static float       s_openloop_duty = 0.0f;
#if (NEUQ_RUNAWAY_GUARD)
static uint16_t    s_runaway_cnt = 0;   /* 判据①计数：输出在推，转速却往反方向跑 */
static uint16_t    s_tgt_cnt     = 0;   /* 判据②计数：目标正、实际却长期为负       */
static uint8_t     s_flip_cnt    = 0;   /* 本次运行里已自动翻符号的次数            */
static uint16_t    s_cool_cnt    = 0;   /* 翻符号后的冷却期，避免来回翻            */
static float       s_meas_last   = 0.0f;/* 上一拍转速，用来看"是不是越跑越远"      */
#endif

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       闭环控制初始化
* 参数说明       void
* 返回参数       void
* 使用示例       NEUQ_Control_Init();
* 备注信息       上电状态 = 停止 + 目标 0 + 默认 PID，必须等上位机下发 START 才动
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Control_Init(void)
{
    NEUQ_Pid_Init(&s_pid);

    /* 指令邮箱初值：与 s_pid 保持一致，保证第一次中断应用时不会把参数改回 0 */
    s_box.target        = 0.0f;
    s_box.kp            = NEUQ_PID_KP_DEFAULT;
    s_box.ki            = NEUQ_PID_KI_DEFAULT;
    s_box.kd            = NEUQ_PID_KD_DEFAULT;
    s_box.kff           = NEUQ_PID_KFF_DEFAULT;
    s_box.slew          = NEUQ_PID_SLEW_DEFAULT;
    s_box.openloop_duty = 0.0f;
    s_box.enable        = 0;
    s_box.openloop      = 0;
    s_box.pid_dirty     = 0;
    s_box.clear_req     = 0;
    s_box.pid_sel       = 0;
    s_box.pid_val       = 0.0f;
    s_box.pid_sel_dirty = 0;
    s_box.seq           = 0;
    s_box_applied       = 0xFFFFFFFFu;     /* 强制第一次中断应用一次邮箱 */

    s_tick         = 0;
    s_target       = 0.0f;
    s_ref          = 0.0f;
    s_enable       = 0;
    s_openloop     = 0;
    s_openloop_duty = 0.0f;

#if (NEUQ_RUNAWAY_GUARD)
    s_runaway_cnt = 0;
    s_tgt_cnt     = 0;
    s_flip_cnt    = 0;
    s_cool_cnt    = 0;
    s_meas_last   = 0.0f;
#endif

    /* 快照先填一遍，避免上位机在首个中断到来前读到随机值 */
    s_snap.target   = 0.0f;
    s_snap.actual   = 0.0f;
    s_snap.kp       = s_pid.kp;
    s_snap.ki       = s_pid.ki;
    s_snap.kd       = s_pid.kd;
    s_snap.output   = 0.0f;
    s_snap.kff      = s_pid.kff;
    s_snap.slew     = s_pid.slew_limit;
    s_snap.enable   = 0;
    s_snap.openloop = 0;
    s_snap.tick     = 0;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       定周期控制中断体（由 TIM6 中断回调调用，周期 NEUQ_CONTROL_PERIOD_MS）
* 参数说明       void
* 返回参数       void
* 使用示例       NEUQ_PIT_1_CALLBACK() 中调用
* 备注信息       本函数必须"短且定长"：只有邮箱应用、编码器读取、PID、PWM 赋值、快照赋值，
*                没有任何 while 等待、没有串口收发、没有打印。
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Control_ISR(void)
{
    float meas;
    float out = 0.0f;

    /*============================== ① 应用主循环下发的指令 ==============================*/
    if (s_box.seq != s_box_applied)
    {
        s_box_applied = s_box.seq;

        s_target = s_box.target;

        if (s_box.pid_dirty)                        /* 更新 PID 参数（停/运行中都允许） */
        {
            NEUQ_Pid_SetGains(&s_pid, s_box.kp, s_box.ki, s_box.kd);
            s_pid.kff        = s_box.kff;
            s_pid.slew_limit = s_box.slew;
            s_box.pid_dirty  = 0;
        }

        if (s_box.clear_req)                        /* 清积分/内部状态                  */
        {
            NEUQ_Pid_Reset(&s_pid);
            s_box.clear_req = 0;
        }

        if (s_box.pid_sel_dirty)                    /* 单参数 PID：只改指定的那一个      */
        {
            switch (s_box.pid_sel)
            {
                case 1:  NEUQ_Pid_SetGains(&s_pid, s_box.pid_val, s_pid.ki,      s_pid.kd);      break;
                case 2:  NEUQ_Pid_SetGains(&s_pid, s_pid.kp,      s_box.pid_val, s_pid.kd);      break;
                case 3:  NEUQ_Pid_SetGains(&s_pid, s_pid.kp,      s_pid.ki,      s_box.pid_val); break;
                default: break;
            }
            s_box.pid_sel_dirty = 0;
        }

        s_openloop      = s_box.openloop;           /* 开环调试模式                     */
        s_openloop_duty = s_box.openloop_duty;

        if (s_box.enable != s_enable)               /* 启停状态变化                     */
        {
            if (s_box.enable)
            {
                NEUQ_Pid_Reset(&s_pid);               /* 启动：调节器归零，从 0 输出平滑起步 */
                s_ref = 0.0f;
#if (NEUQ_RUNAWAY_GUARD)
                s_flip_cnt     = 0;                   /* 每次启动重新给自动纠错机会          */
                s_runaway_cnt  = 0;
                s_tgt_cnt      = 0;
                s_cool_cnt     = 0;
                s_meas_last    = 0.0f;
#endif
            }
            else
            {
                NEUQ_Pid_Reset(&s_pid);               /* 停止：清积分，避免下次启动带旧积分   */
            }
            s_enable = s_box.enable;
        }
    }

    /*============================== ② 目标参考（可选斜坡/目标规划） ==============================*/
#if (NEUQ_TARGET_RAMP_ENABLE)
    {
        float dmax = NEUQ_TARGET_RAMP_RATE * NEUQ_CONTROL_DT;
        float de   = s_target - s_ref;
        if      (de >  dmax) { s_ref += dmax; }
        else if (de < -dmax) { s_ref -= dmax; }
        else                 { s_ref  = s_target; }
    }
#else
    s_ref = s_target;                               /* 默认直接阶跃，响应最真实 */
#endif

    /*============================== ③ 编码器测速（每个周期都要读并清零） ==============================*/
    /*  注意：无论电机是否运行都要读，否则停机期间计数会一直累积，
     *        下次启动的第一次采样会得到一个巨大的假转速。 */
    meas = NEUQ_Speed_Update();

    /*============================== ④ 闭环计算 ==============================*/
    if (s_openloop)
    {
        out = s_openloop_duty;                      /* 开环调试：直接给占空比 */
    }
    else if (s_enable)
    {
        out = NEUQ_Pid_Update(&s_pid, s_ref, meas, NEUQ_CONTROL_DT);
    }
    else
    {
        out = 0.0f;
    }

    /*============================== ④.5 失控保护 / 自动符号纠错 ==============================*/
#if (NEUQ_RUNAWAY_GUARD)
    /*  两条判据，任一成立就认为"电机转向"和"编码器读数方向"对不上（正反馈）：
     *    ① 输出在推、转速却往反方向跑（持续 0.5s）—— 发散型；
     *    ② 目标为正、实际却长期为负（或反过来，持续 2s）—— 卡在"反向平衡点"型。
     *       典型现象：目标 61.7，i1 稳定在 -140、i5 稳定在 4600 —— 看着不抖，
     *       其实是闭环稳定在了一个**相反**的解上，光看波形很容易误判成"参数问题"。
     *  处理：头 NEUQ_RUNAWAY_MAX_FLIP 次自动把测速符号取反 + 清积分继续跑；
     *        翻完还不对就停机（enable=0），等上位机重新 START。 */
    {
        float dmeas   = meas - s_meas_last;
        float abs_out = (out < 0.0f) ? (-out) : out;
        int   mismatch = 0;
        int   opp_tgt  = 0;
        int   do_fix   = 0;
        int   fix_tgt  = 0;      /* 1 = 由"目标与实际反号"触发（可以翻符号）；0 = 发散（只停机） */

        if ((out > 0.0f) && (dmeas < -0.1f)) { mismatch = 1; }
        if ((out < 0.0f) && (dmeas >  0.1f)) { mismatch = 1; }

        if (((s_ref > 10.0f) && (meas < -20.0f)) ||
            ((s_ref < -10.0f) && (meas > 20.0f)))
        {
            opp_tgt = 1;
        }

        if (s_cool_cnt > 0u)
        {
            /* 刚翻过符号：转速读数会突跳一下，冷却一段时间再判断 */
            s_cool_cnt--;
            s_runaway_cnt = 0;
            s_tgt_cnt     = 0;
        }
        else
        {
            /* 判据①：输出够大(>30%)、转速不接近 0，却在往输出相反方向走 */
            if (mismatch && (abs_out > (NEUQ_MOTOR_PWM_MAX * 0.30f)) &&
                ((meas > 15.0f) || (meas < -15.0f)))
            {
                s_runaway_cnt++;
                if (s_runaway_cnt >= NEUQ_RUNAWAY_TICKS) { do_fix = 1; s_runaway_cnt = 0; }
            }
            else
            {
                s_runaway_cnt = 0;
            }

            /* 判据②：目标与实际符号相反且持续 2 秒（给正常换向留足时间） */
            if (opp_tgt)
            {
                s_tgt_cnt++;
                if (s_tgt_cnt >= NEUQ_RUNAWAY_TICKS2) { do_fix = 1; fix_tgt = 1; s_tgt_cnt = 0; }
            }
            else
            {
                s_tgt_cnt = 0;
            }

            if (do_fix)
            {
                s_cool_cnt = NEUQ_RUNAWAY_TICKS2;

                /* 只有"目标正、实际长期为负"这种明确的闭环反号，才自动翻测速符号；
                 * 判据①那种发散（多半是 Kp 太大在饱和振荡，或者方向脚根本没接）
                 * 不翻符号，直接停机，免得把本来对的符号翻错。 */
                if (fix_tgt && (s_flip_cnt < NEUQ_RUNAWAY_MAX_FLIP))
                {
                    /* 自动纠正：把测速符号取反，等价于手发 0x14 命令 */
                    NEUQ_Speed_SetDirSign(-NEUQ_Speed_GetDirSign());
                    NEUQ_Pid_Reset(&s_pid);
                    s_flip_cnt++;
                }
                else
                {
                    /* 兜底：停机，等重新启动 */
                    s_enable       = 0;
                    s_box.enable   = 0;
                    s_box.openloop = 0;
                    NEUQ_Pid_Reset(&s_pid);
                    out = 0.0f;
                }
            }
        }

        s_meas_last = meas;
    }
#endif

    /*============================== ⑤ 输出到电机 ==============================*/
    if (s_enable || s_openloop)
    {
        NEUQ_Motor_SetOutput(out);                    /* 运行中输出 0 = 关断，不刹车 */
    }
    else
    {
        NEUQ_Motor_Stop();                            /* 停机：按配置刹车或滑行      */
    }

    /*============================== ⑥ 更新回传快照 ==============================*/
    s_snap.target   = s_ref;
    s_snap.actual   = meas;
    s_snap.kp       = s_pid.kp;
    s_snap.ki       = s_pid.ki;
    s_snap.kd       = s_pid.kd;
    s_snap.kff      = s_pid.kff;
    s_snap.slew     = s_pid.slew_limit;
    s_snap.output   = NEUQ_Motor_GetOutput();
    s_snap.enable   = s_enable;
    s_snap.openloop = s_openloop;

    s_tick++;
    s_snap.tick = s_tick;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       取回传快照
* 参数说明       dst   输出缓冲
* 返回参数       void
* 使用示例       NEUQ_Snapshot s; NEUQ_Control_GetSnapshot(&s);
* 备注信息       关中断拷贝结构体（约十几条指令），保证一整帧数据来自同一个控制周期。
*                关中断时间远小于编码器计数/串口字节的处理时间，不会影响测速与通信。
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Control_GetSnapshot(NEUQ_Snapshot *dst)
{
    __disable_irq();
    *dst = s_snap;
    __enable_irq();
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       取控制节拍计数
* 参数说明       void
* 返回参数       节拍数（每个控制周期 +1）
* 使用示例       uint32_t t = NEUQ_Control_GetTick();
*------------------------------------------------------------------------------------------------------------------*/
uint32_t NEUQ_Control_GetTick(void)
{
    return s_tick;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       取当前运行状态（0 = 停止/未启动，1 = 闭环运行中）
* 备注信息       通信模块用它实现"PID 参数只在停止状态下允许修改"的题目要求
*------------------------------------------------------------------------------------------------------------------*/
uint8_t NEUQ_Control_GetState(void)
{
    return s_snap.enable;
}

/*===================================================================================================================
 *                            以下是通信模块调用的指令接口（主循环上下文）
 *=================================================================================================================*/

/*  指令       启动闭环：退出开环调试，清 PID 状态，输出从 0 平滑建立   */
void NEUQ_Control_ReqStart(void)
{
    s_box.openloop = 0;
    s_box.enable   = 1;
    s_box.seq++;
}

/*  指令       停止闭环：退出开环调试，输出清零并按配置刹车            */
void NEUQ_Control_ReqStop(void)
{
    s_box.enable        = 0;
    s_box.openloop      = 0;
    s_box.openloop_duty = 0.0f;
    s_box.seq++;
}

/*  指令       修改目标转速（运行中随时可改，自动限幅）                */
void NEUQ_Control_ReqTarget(float target)
{
    /*  防护：滑条/上位机若因丢字节解析出 NaN 或无穷大，直接丢弃这一帧，
     *  否则 target 会变成 NaN，PID 输出全是 NaN，曲线看起来"完全没反应"。*/
    if (target != target) { return; }                    /* NaN  */
    if ((target >  1.0e9f) || (target < -1.0e9f)) { return; }   /* Inf */

    if (target >  NEUQ_TARGET_MAX) { target =  NEUQ_TARGET_MAX; }
    if (target <  NEUQ_TARGET_MIN) { target =  NEUQ_TARGET_MIN; }

#if (NEUQ_MOTOR_ALLOW_REVERSE == 0)
    /* 只正转：负目标按 0 处理，这样 i0 显示的值和电机实际行为一致 */
    if (target < 0.0f) { target = 0.0f; }
#endif

    s_box.target = target;
    s_box.seq++;
}

/*  指令       修改 PID 参数（只需 Kp/Ki/Kd，与题目的帧格式对应）      */
void NEUQ_Control_ReqPid(float kp, float ki, float kd)
{
    if (kp < 0.0f) { kp = 0.0f; }
    if (ki < 0.0f) { ki = 0.0f; }
    if (kd < 0.0f) { kd = 0.0f; }
    s_box.kp        = kp;
    s_box.ki        = ki;
    s_box.kd        = kd;
    s_box.pid_dirty = 1;
    s_box.seq++;
}

/*  指令       修改前馈增益 / 输出变化率限幅（辅助参数，可选）          */
void NEUQ_Control_ReqGain2(float kff, float slew)
{
    if (kff  < 0.0f)     { kff  = 0.0f; }
    if (kff  > 100.0f)   { kff  = 100.0f; }
    if (slew < 0.0f)     { slew = 0.0f; }
    if (slew > NEUQ_MOTOR_PWM_MAX) { slew = NEUQ_MOTOR_PWM_MAX; }
    s_box.kff       = kff;
    s_box.slew      = slew;
    s_box.pid_dirty = 1;
    s_box.seq++;
}

/*  指令       清 PID 积分与内部状态（不改变启停状态）                  */
void NEUQ_Control_ReqClear(void)
{
    s_box.clear_req = 1;
    s_box.seq++;
}

/*  指令       单参数修改 PID（专给 VOFA+ 滑条用）
 *  参数说明     sel  1=修改 Kp，2=修改 Ki，3=修改 Kd
 *               val  该参数的新值（负值按 0 处理）
 *  备注信息     为什么需要它：VOFA+ 滑条的参数占位符 %% 一次只能替换【一个】float，
 *               而 0x01 号命令一帧要带 kp+ki+kd 三个 float，绑不了三个滑条。
 *               所以拆成三条单参数命令，滑条各绑一条，互不影响。            */
void NEUQ_Control_ReqPidOne(uint8_t sel, float val)
{
    if (val < 0.0f) { val = 0.0f; }
    if (sel < 1u || sel > 3u) { return; }

    s_box.pid_sel       = sel;
    s_box.pid_val       = val;
    s_box.pid_sel_dirty = 1;
    s_box.seq++;
}

/*  指令       开环调试：直接给占空比，用于确认接线/电机转向/测速符号。
 *             为安全起见，收到该命令时自动进入"停止"状态并退出闭环。  */
void NEUQ_Control_ReqOpenloop(float duty)
{
    if (duty >  NEUQ_MOTOR_PWM_MAX) { duty =  NEUQ_MOTOR_PWM_MAX; }
    if (duty < -NEUQ_MOTOR_PWM_MAX) { duty = -NEUQ_MOTOR_PWM_MAX; }

    s_box.enable        = 0;
    s_box.openloop      = 1;
    s_box.openloop_duty = duty;
    s_box.seq++;
}
