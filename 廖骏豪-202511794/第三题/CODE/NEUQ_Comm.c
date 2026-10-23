/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_Comm
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
********************************************************************************************************************/
#include "NEUQ_Config.h"
#include "NEUQ_Comm.h"
#include "NEUQ_Control.h"
#include "NEUQ_Speed.h"

/* NEUQ_Uart.h 只声明了 init / write_xxx；使能接收中断的函数定义在 NEUQ_Uart.c 里，
   但没有在头文件中声明，这里补充声明后即可正常调用（不修改原库文件）。 */
extern void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

/*-------------------------------------------------------------------------------------------------------------------
* 接收环形缓冲：中断只写 head，主循环只写 tail，单生产者单消费者，无需临界区
*------------------------------------------------------------------------------------------------------------------*/
static volatile uint8_t  s_rx_buf[NEUQ_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;

static NEUQ_CommStats s_stats;                    /* 通信统计                     */

#define NEUQ_COMM_POLL_MAX_BYTES   512            /* 单次轮询最多处理的字节数。
                                                   * 拖滑条时是突发连续帧，一次多消费一些，
                                                   * 避免积压导致缓冲溢出。                     */

/*-------------------------------------------------------------------------------------------------------------------
 * 接收解析状态机（不带 CRC，AA 55 和 CC DD 两种帧头都按无校验处理）
 *
 *   帧格式：<HEAD0> <HEAD1> CMD LEN DATA[LEN]
 *       AA 55 …  和  CC DD …  等价，都不带校验字节。
 *   为什么去掉：VOFA+ 的命令/滑条占位符 %% 无法重算 CRC，
 *   带校验时滑条永远发不出合法帧，所以整条下行链路统一不带校验。
 *   上行（回传）本来就是 FireWater 文本，也没有校验。
 *
 *   注意：数据域收满的那个字节就要分发，不能等下一个字节来触发。
 *   早期把分发单独做成一个状态，单条命令永远等不到触发、多条命令又会把下一帧的帧头吃掉。
 *   现在收满即分发，然后立刻回到等帧头。
 *------------------------------------------------------------------------------------------------------------------*/
typedef enum
{
    NEUQ_ST_HEAD0 = 0,        /* 等帧头 0xAA（或 0xCC）        */
    NEUQ_ST_HEAD1,            /* 等帧头第二字节 0x55（或 0xDD）*/
    NEUQ_ST_CMD,              /* 命令字                       */
    NEUQ_ST_LEN,              /* 长度                         */
    NEUQ_ST_DATA              /* 数据域（收满即分发）         */
} NEUQ_ParseState;

static NEUQ_ParseState s_state = NEUQ_ST_HEAD0;
static uint8_t  s_pdata[NEUQ_FRAME_MAX_DATA];     /* 数据域缓存                   */
static uint8_t  s_pcmd  = 0;                    /* 命令字缓存                   */
static uint8_t  s_plen  = 0;                    /* 数据长度                     */
static uint8_t  s_pidx  = 0;                    /* 数据接收下标                 */
static uint8_t  s_h0    = 0;                    /* 本帧第一个帧头字节（0xAA/0xCC）*/
static uint32_t s_pstart_tick = 0;              /* 本帧第一个字节所在的控制节拍 */

/* 回传节拍控制 */
static uint32_t s_last_upload_tick = 0;
static uint8_t  s_force_upload    = 0;          /* 1 = 收到命令后立即回传一次   */
static uint32_t s_force_tick      = 0;          /* 命令生效前所在的控制节拍     */

#define NEUQ_FRAME_TIMEOUT_TICKS  ((NEUQ_FRAME_TIMEOUT_MS + NEUQ_CONTROL_PERIOD_MS - 1) / NEUQ_CONTROL_PERIOD_MS)

/*===================================================================================================================
*                                        浮点打包小工具
*=================================================================================================================*/
/*  float -> 小端 4 字节（STM32 本身是小端，用联合体避免指针强转的隐患） */
static void NEUQ_PutFloat(uint8_t *p, float v)
{
    union { float f; uint8_t b[4]; } u;
    u.f = v;
    p[0] = u.b[0];
    p[1] = u.b[1];
    p[2] = u.b[2];
    p[3] = u.b[3];
}

/*  小端 4 字节 -> float */
static float NEUQ_GetFloat(const uint8_t *p)
{
    union { float f; uint8_t b[4]; } u;
    u.b[0] = p[0];
    u.b[1] = p[1];
    u.b[2] = p[2];
    u.b[3] = p[3];
    return u.f;
}

/*===================================================================================================================
*                                            初始化 / 接收中断
*=================================================================================================================*/
/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       通信初始化
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Comm_Init(void)
{
    NEUQ_Uart_init(NEUQ_COMM_UART, NEUQ_COMM_BAUD);

    s_rx_head = 0;
    s_rx_tail = 0;
    s_state   = NEUQ_ST_HEAD0;
    s_pidx    = 0;
    s_h0      = 0;

    s_stats.rx_ok       = 0;
    s_stats.rx_bad_crc  = 0;                     /* 已取消 CRC 校验，该计数恒为 0 */
    s_stats.rx_bad_len  = 0;
    s_stats.rx_timeout  = 0;
    s_stats.rx_overflow = 0;
    s_stats.tx_frames   = 0;

    s_force_upload = 0;
    s_last_upload_tick = NEUQ_Control_GetTick();

    /* 打开接收中断，之后每收到一个字节就会进 TS_UART_x_RX_CALLBACK */
    NEUQ_Uart_Enable_RX_Interrupt(NEUQ_COMM_UART, NEUQ_COMM_RX_IRQ_PRIO);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       串口接收中断：仅入环形缓冲
* 参数说明       dat   收到的字节
* 返回参数       void
* 备注信息       缓冲区满时丢弃新字节而不是覆盖旧数据（宁可丢也要保证状态机不被破坏）
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Comm_RxByte(uint8_t dat)
{
    uint16_t next = (uint16_t)((s_rx_head + 1u) & (NEUQ_RX_BUF_SIZE - 1u));

    if (next != s_rx_tail)
    {
        s_rx_buf[s_rx_head] = dat;
        s_rx_head = next;
    }
    else
    {
        s_stats.rx_overflow++;
    }
}

/*===================================================================================================================
*                                              主循环：解析
*=================================================================================================================*/
/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       解析出一个完整帧后的分发处理
* 参数说明       cmd   命令字；data 数据域；len 数据长度
* 返回参数       void
* 备注信息       只调用 NEUQ_Control_ReqXxx 写指令邮箱，真正的参数应用交给控制中断
*------------------------------------------------------------------------------------------------------------------*/
static void NEUQ_Comm_Dispatch(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    switch (cmd)
    {
        case NEUQ_CMD_SET_PID:                        /* Kp, Ki, Kd */
            if (len == 12u)
            {
                /* 题目要求"PID 参数可在停止状态下修改" —— 这里在停止/运行中都接受，
                 * 属于题目要求的超集，不会违反规则。
                 * 早期只在停止时接受，运行中改参数会被丢掉，调参很别扭，所以放开。 */
                NEUQ_Control_ReqPid(NEUQ_GetFloat(&data[0]),
                                  NEUQ_GetFloat(&data[4]),
                                  NEUQ_GetFloat(&data[8]));
            }
            break;

        case NEUQ_CMD_SET_TARGET:                     /* 目标转速 */
            if (len == 4u)
            {
                NEUQ_Control_ReqTarget(NEUQ_GetFloat(&data[0]));
            }
            break;

        case NEUQ_CMD_START:
            if (len == 0u) { NEUQ_Control_ReqStart(); }
            break;

        case NEUQ_CMD_STOP:
            if (len == 0u) { NEUQ_Control_ReqStop(); }
            break;

        case NEUQ_CMD_SET_GAIN2:                      /* 前馈增益 + 输出变化率限幅 */
            if (len == 8u)
            {
                NEUQ_Control_ReqGain2(NEUQ_GetFloat(&data[0]),
                                    NEUQ_GetFloat(&data[4]));
            }
            break;

        case NEUQ_CMD_CLEAR:
            if (len == 0u) { NEUQ_Control_ReqClear(); }
            break;

        case NEUQ_CMD_QUERY:
            /* 空命令，仅触发一次立即回传 */
            break;

        case NEUQ_CMD_OPENLOOP:                       /* 开环调试占空比 */
            if (len == 4u)
            {
                NEUQ_Control_ReqOpenloop(NEUQ_GetFloat(&data[0]));
            }
            break;

        /* ---- 单参数 PID：专给 VOFA+ 滑条用（每条只改一个参数，其余不动） ----
         * 停止/运行中都接受（题目只要求"停止状态下可改"，这是超集）。 */
        case NEUQ_CMD_SET_KP_ONE:
            if (len == 4u)
            {
                NEUQ_Control_ReqPidOne(1u, NEUQ_GetFloat(&data[0]));
            }
            break;

        case NEUQ_CMD_SET_KI_ONE:
            if (len == 4u)
            {
                NEUQ_Control_ReqPidOne(2u, NEUQ_GetFloat(&data[0]));
            }
            break;

        case NEUQ_CMD_SET_KD_ONE:
            if (len == 4u)
            {
                NEUQ_Control_ReqPidOne(3u, NEUQ_GetFloat(&data[0]));
            }
            break;

        /* ---- 运行时改测速方向符号：现场排查正反馈飞车用 ----
         * 发 +1.0 -> 计数正向读作正转速；发 -1.0 -> 取反。
         * 只影响 i1 的符号，不重启、不影响 PWM。            */
        case NEUQ_CMD_SET_DIRSIGN:
            if (len == 4u)
            {
                NEUQ_Speed_SetDirSign(NEUQ_GetFloat(&data[0]));
            }
            break;

        default:
            return;                                 /* 未知命令：不回传，避免误触发 */
    }

    /* 命令解析成功：标记"立即回传一次"。
     * 回传要等一个控制节拍之后再发，这样上传的是参数已经生效后的真实值，
     * 满足"上位机显示以回传值为准 / 回传值与实际使用值一致"的要求。 */
    s_force_upload = 1;
    s_force_tick   = NEUQ_Control_GetTick();
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       逐字节解析（可在任意字节处中断，下次继续）
* 参数说明       b   收到的字节
* 返回参数       void
* 备注信息       状态机天然支持：
*                   · 一帧被分成多次到达（中间停多久都行，只要不超过 NEUQ_FRAME_TIMEOUT_MS）
*                   · 多帧连续到达（数据收满后立刻分发并回到等帧头状态）
*                   · 帧间噪声（反复遇到 0xAA 会一直停在等 0x55 状态，能自我重新同步）
*------------------------------------------------------------------------------------------------------------------*/
static void NEUQ_Comm_ParseByte(uint8_t b)
{
    /* ---------- 半帧超时：超过设定时间没有收完，说明是噪声或上位机中途断开，丢弃重新同步 ---------- */
    if (s_state != NEUQ_ST_HEAD0)
    {
        if ((uint32_t)(NEUQ_Control_GetTick() - s_pstart_tick) > NEUQ_FRAME_TIMEOUT_TICKS)
        {
            s_state = NEUQ_ST_HEAD0;
            s_stats.rx_timeout++;
        }
    }

    switch (s_state)
    {
        case NEUQ_ST_HEAD0:
            /* 两种帧头都接受：AA 55（旧命令/上位机工具）和 CC DD（VOFA+ 滑条） */
            if ((b == NEUQ_FRAME_HEAD0) || (b == NEUQ_FRAME2_HEAD0))
            {
                s_h0          = b;                        /* 记住是哪种帧头 */
                s_state       = NEUQ_ST_HEAD1;
                s_pstart_tick = NEUQ_Control_GetTick();
            }
            break;

        case NEUQ_ST_HEAD1:
            if (((s_h0 == NEUQ_FRAME_HEAD0)  && (b == NEUQ_FRAME_HEAD1)) ||
                ((s_h0 == NEUQ_FRAME2_HEAD0) && (b == NEUQ_FRAME2_HEAD1)))
            {
                s_state = NEUQ_ST_CMD;
            }
            else if ((b == NEUQ_FRAME_HEAD0) || (b == NEUQ_FRAME2_HEAD0))
            {
                s_h0 = b;                                 /* 连续帧头：把当前字节当作新的帧头 */
            }
            else
            {
                s_state = NEUQ_ST_HEAD0;
            }
            break;

        case NEUQ_ST_CMD:
            s_pcmd = b;
            s_state = NEUQ_ST_LEN;
            break;

        case NEUQ_ST_LEN:
            s_plen = b;
            if (s_plen > NEUQ_FRAME_MAX_DATA)
            {
                s_stats.rx_bad_len++;
                s_state = NEUQ_ST_HEAD0;
                break;
            }
            s_pidx = 0;
            if (s_plen == 0u)
            {
                /* 无数据域的命令（启动/停止/清状态/查询）在 LEN 这一拍就能分发 */
                s_stats.rx_ok++;
                NEUQ_Comm_Dispatch(s_pcmd, s_pdata, 0u);
                s_state = NEUQ_ST_HEAD0;
            }
            else
            {
                s_state = NEUQ_ST_DATA;
            }
            break;

        case NEUQ_ST_DATA:
            s_pdata[s_pidx++] = b;
            if (s_pidx >= s_plen)
            {
                /* 收满就当拍分发，然后回到等帧头 */
                s_stats.rx_ok++;
                NEUQ_Comm_Dispatch(s_pcmd, s_pdata, s_plen);
                s_state = NEUQ_ST_HEAD0;
            }
            break;

        default:
            s_state = NEUQ_ST_HEAD0;
            break;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       通信轮询（主循环调用）
* 参数说明       void
* 返回参数       void
* 备注信息       每次最多处理 NEUQ_COMM_POLL_MAX_BYTES 个字节，避免高波特率下长时间占住主循环
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Comm_Poll(void)
{
    uint16_t n = 0;

    while ((s_rx_tail != s_rx_head) && (n < NEUQ_COMM_POLL_MAX_BYTES))
    {
        uint8_t b = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) & (NEUQ_RX_BUF_SIZE - 1u));
        NEUQ_Comm_ParseByte(b);
        n++;
    }
}

/*===================================================================================================================
*                                              主循环：回传
*=================================================================================================================*/
/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       浮点数 -> 字符串（不依赖 printf / sprintf）
* 参数说明       dst  输出缓冲（至少 16 字节）
* 参数说明       v    数值
* 参数说明       dec  小数位数
* 返回参数       写入的字符数
* 备注信息       microlib 下 printf 带浮点会撑大十几 KB，所以手写一个格式化。
*                支持负数、NaN 归零、上限 1e8 防溢出。
*------------------------------------------------------------------------------------------------------------------*/
static int NEUQ_FwPutFloat(char *dst, float v, int dec)
{
    int   k = 0;
    long  ip;
    long  frac = 0;
    long  scale = 1;
    char  tmp[12];
    int   n = 0, i;
    float av = v;

    if (av != av) { av = 0.0f; }                       /* NaN -> 0        */
    if (av < 0.0f) { dst[k++] = '-'; av = -av; }
    if (av > 1.0e8f) { av = 1.0e8f; }                  /* 限幅，防 long 溢出 */

    for (i = 0; i < dec; i++) { scale *= 10L; }

    ip   = (long)av;
    frac = (long)((av - (float)ip) * (float)scale + 0.5f);
    if (frac >= scale) { frac -= scale; ip += 1L; }

    if (ip == 0L) { tmp[n++] = '0'; }
    else { while ((ip > 0L) && (n < (int)sizeof(tmp))) { tmp[n++] = (char)('0' + (int)(ip % 10L)); ip /= 10L; } }
    while (n > 0) { dst[k++] = tmp[--n]; }

    if (dec > 0)
    {
        dst[k++] = '.';
        for (i = dec - 1; i >= 0; i--)
        {
            long d = 1L;
            int  j;
            for (j = 0; j < i; j++) { d *= 10L; }
            dst[k++] = (char)('0' + (int)((frac / d) % 10L));
        }
    }

    return k;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       组帧并发送一帧
* 参数说明       snap  当前生效数据快照
* 返回参数       void
* 备注信息       三种格式（由 NEUQ_UPLINK_MODE 选择）：
*                 0 = JustFloat     N 个 float + 帧尾 00 00 80 7F（二进制）
*                 1 = 自定义二进制帧 AA 55 81 LEN DATA（无校验）
*                 2 = FireWater     文本："目标,实际,Kp,Ki,Kd\r\n"
*------------------------------------------------------------------------------------------------------------------*/
static void NEUQ_Comm_SendFrame(const NEUQ_Snapshot *snap)
{
    uint8_t  buf[96];
    uint16_t n = 0;
    uint8_t  data[28];
    uint8_t  dl = 0;

    /* 先按统一顺序打包负载：目标转速 / 实际转速 / Kp / Ki / Kd [/ 输出 / 使能] */
    NEUQ_PutFloat(&data[dl], snap->target); dl = (uint8_t)(dl + 4u);
    NEUQ_PutFloat(&data[dl], snap->actual); dl = (uint8_t)(dl + 4u);
    NEUQ_PutFloat(&data[dl], snap->kp);     dl = (uint8_t)(dl + 4u);
    NEUQ_PutFloat(&data[dl], snap->ki);     dl = (uint8_t)(dl + 4u);
    NEUQ_PutFloat(&data[dl], snap->kd);     dl = (uint8_t)(dl + 4u);
#if (NEUQ_UPLINK_WITH_STATE)
    NEUQ_PutFloat(&data[dl], snap->output); dl = (uint8_t)(dl + 4u);
    NEUQ_PutFloat(&data[dl], (float)snap->enable); dl = (uint8_t)(dl + 4u);
#endif

#if (NEUQ_UPLINK_MODE == NEUQ_UPLINK_MODE_FIREWATER)
    /* ---------------- FireWater：纯文本，逗号分隔，回车换行结尾 ----------------
     * VOFA+ 里"数据格式"选 FireWater，"通道数"设 5 即可直接出曲线。
     * 每秒约 50 帧、每帧 ~30 字节 -> 1500B/s，115200 下占用很小。       */
    {
        char *p = (char *)buf;
        int   k = 0;

        k += NEUQ_FwPutFloat(p + k, snap->target, 2); p[k++] = ',';
        k += NEUQ_FwPutFloat(p + k, snap->actual, 2); p[k++] = ',';
        k += NEUQ_FwPutFloat(p + k, snap->kp,     2); p[k++] = ',';
        k += NEUQ_FwPutFloat(p + k, snap->ki,     2); p[k++] = ',';
        k += NEUQ_FwPutFloat(p + k, snap->kd,     2);
#if (NEUQ_UPLINK_WITH_STATE)
        p[k++] = ',';
        k += NEUQ_FwPutFloat(p + k, snap->output, 1);
        p[k++] = ',';
        k += NEUQ_FwPutFloat(p + k, (float)snap->enable, 0);
#endif
        p[k++] = '\r';
        p[k++] = '\n';
        n = (uint16_t)k;
    }
#elif (NEUQ_UPLINK_MODE == NEUQ_UPLINK_MODE_FRAME)
    {
        uint8_t i;

        /* 自定义二进制回传帧（同样无校验，和下行保持一致）：
         *   AA 55 81 LEN DATA[LEN]                                        */
        buf[n++] = NEUQ_FRAME_HEAD0;
        buf[n++] = NEUQ_FRAME_HEAD1;
        buf[n++] = NEUQ_CMD_FEEDBACK;
        buf[n++] = dl;

        for (i = 0; i < dl; i++) { buf[n++] = data[i]; }
    }
#else
    {
        uint8_t i;
        for (i = 0; i < dl; i++) { buf[n++] = data[i]; }
        /* VOFA+ JustFloat 帧尾 */
        buf[n++] = 0x00u;
        buf[n++] = 0x00u;
        buf[n++] = 0x80u;
        buf[n++] = 0x7Fu;
    }
#endif

    NEUQ_Uart_write_data(NEUQ_COMM_UART, buf, (uint32_t)n);

    s_stats.tx_frames++;
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       按节拍回传（主循环调用）
* 参数说明       void
* 返回参数       void
* 备注信息       节拍基准是控制中断的 tick，与主循环执行快慢无关：
*                 · 平时每 NEUQ_UPLOAD_EVERY_N_TICKS 个周期回传一次（默认 2 -> 100Hz，VOFA+ 曲线足够平滑）
*                 · 收到命令后额外插一次立即回传，但要等一个控制节拍，确保回传的是"已经生效"的参数
*------------------------------------------------------------------------------------------------------------------*/
void NEUQ_Comm_Upload(void)
{
    NEUQ_Snapshot snap;

    NEUQ_Control_GetSnapshot(&snap);

    if (s_force_upload)
    {
        /* 命令刚下发时控制中断可能还没应用，等下一拍再回传，保证回传值与实际使用值一致 */
        if (snap.tick == s_force_tick)
        {
            return;
        }
        s_force_upload = 0;
    }
    else if ((uint32_t)(snap.tick - s_last_upload_tick) < NEUQ_UPLOAD_EVERY_N_TICKS)
    {
        return;
    }

    s_last_upload_tick = snap.tick;
    NEUQ_Comm_SendFrame(&snap);
}

/*-------------------------------------------------------------------------------------------------------------------
* 函数简介       读取通信统计
*------------------------------------------------------------------------------------------------------------------*/
const NEUQ_CommStats *NEUQ_Comm_GetStats(void)
{
    return &s_stats;
}
