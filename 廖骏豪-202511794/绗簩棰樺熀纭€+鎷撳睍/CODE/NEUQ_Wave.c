/*
 * NEUQ_Wave.c
 * 把选中轴的原始值和滤波值成对发给上位机
 *
 * 发送速率的做法：
 *   采样固定500Hz，但上传速率有50/100/200/500几档，不一定能整除。
 *   所以用相位累加，而不是"每N个点发一次"：
 *       相位 += 目标速率，相位 >= 采样率时发一帧并把相位减掉采样率
 *   这样200Hz这种2.5分频也能长期精确，不会越跑越偏。
 */
#include "NEUQ_lib_include.h"
#include "NEUQ_Setting.h"
#include "NEUQ_IMU.h"
#include "NEUQ_Wave.h"

static float    s_phase = 0.0f;     // 速率控制的相位累加器
static uint32_t s_frames = 0;       // 已经发出去的帧数
static uint8_t  s_tx_fail = 0;      // 连续发失败的帧数
static uint8_t  s_uart_ok = 1;      // 1正常，0串口超时已经自动停发

// 把上传串口编号换成寄存器基地址
// 这里故意直接操作寄存器，不用库里的NEUQ_Uart_write_byte()。
// 库函数是 while(!(USARTx->SR & USART_SR_TXE)); 没有超时，
// 万一USART不再置TXE，主循环就永远停在这一行，采集、滤波、按键、刷屏全部停摆，
// 而且拔线也不会恢复，必须复位。下面这个写法带超时，等不到就丢掉这一帧。
static USART_TypeDef *Wave_UartInstance(void)
{
    switch(NEUQ_WAVE_UART)
    {
        case UART_1: return USART2;     // PA2=TX PA3=RX
        case UART_2: return USART3;     // PB10/PB11
        case UART_3: return UART4;      // PC10/PC11
        case UART_0:
        default:     return USART1;     // PA9/PA10
    }
}

// timeout_us微秒内等TXE，超时返回0
static uint8_t Wave_WaitTxe(USART_TypeDef *uart, uint32_t timeout_us)
{
    uint32_t t0  = DWT->CYCCNT;
    uint32_t lim = (SystemCoreClock / 1000000U) * timeout_us;

    while((uart->SR & USART_SR_TXE) == 0)
    {
        if((uint32_t)(DWT->CYCCNT - t0) > lim)
        {
            return 0;
        }
    }
    return 1;
}

// 往指定USART发一串字节，带超时，等不到就返回0
static uint8_t Wave_WriteTo(USART_TypeDef *uart, const uint8_t *pdata, uint32_t len)
{
    uint32_t i;

    for(i = 0; i < len; i++)
    {
        if(Wave_WaitTxe(uart, NEUQ_WAVE_TX_TIMEOUT_US) == 0)
        {
            return 0;
        }
        uart->DR = pdata[i];
    }
    return 1;
}

static uint8_t Wave_WriteBounded(const uint8_t *pdata, uint32_t len)
{
    return Wave_WriteTo(Wave_UartInstance(), pdata, len);
}

// 连续失败到上限就把上传自动关掉，宁可没波形也不能让单片机卡死
static void Wave_ReportResult(uint8_t ok)
{
    if(ok)
    {
        s_tx_fail = 0;
        s_uart_ok = 1;
        s_frames++;                     // 只有真发出去了才计数
    }
    else
    {
        if(s_tx_fail < 255) s_tx_fail++;
        if(s_tx_fail >= NEUQ_WAVE_TX_FAIL_LIMIT)
        {
            s_uart_ok = 0;
            g_setting.wave_enable = 0;  // 页面上的Send会自己变回OFF
        }
    }
}

void NEUQ_Wave_Init(void)
{
    NEUQ_Uart_init(NEUQ_WAVE_UART, NEUQ_WAVE_BAUD);
    s_phase   = 0.0f;
    s_frames  = 0;
    s_tx_fail = 0;
    s_uart_ok = 1;
}

void NEUQ_Wave_ResetPhase(void)
{
    s_phase   = 0.0f;
    s_tx_fail = 0;                      // 重新开始时清掉旧的失败计数，让它有机会自愈
    s_uart_ok = 1;
}

// 上电自检，发一帧全0的合法数据，返回1正常0串口是坏的
// 这一帧在VOFA+上只是t=0处的一个(0,0)点，不影响后面看波形。
// 它只能证明单片机这边发出去了，证明不了线接对没有。
uint8_t NEUQ_Wave_SelfTest(void)
{
    static const char test_frame[] = "0.0,0.0\n";
    uint8_t ok = Wave_WriteBounded((const uint8_t *)test_frame, sizeof(test_frame) - 1);

    s_uart_ok = ok;
    s_tx_fail = 0;
    return ok;
}

uint8_t NEUQ_Wave_IsUartOk(void)
{
    return s_uart_ok;
}

// 往上传串口打一行字，自带超时
// 上电自检信息用它打，这样不管NEUQ_WAVE_UART选的哪个口，
// 在看波形的那个串口助手上都能看到这几行
void NEUQ_Wave_Log(const char *text)
{
    uint32_t n = 0;

    if(text == 0) return;
    while(text[n] != '\0') n++;
    if(n == 0) return;

    Wave_WriteBounded((const uint8_t *)text, n);
}

// 上电时往PA9和PC10各喊一声，看串口助手上出来哪一行，就知道线接在哪个脚上
// 故意不探PA2：本板PA2经R5接到了电机信号座，那个脚上已经有信号了，
// 再把它改成串口推挽输出等于两个源对打，没必要冒这个险。
void NEUQ_Wave_PortProbe(void)
{
    static const char msg_pa9[]  = "\r\n[PROBE] USART1 TX = PA9  -> use NEUQ_WAVE_UART = UART_0\r\n";
    static const char msg_pc10[] = "\r\n[PROBE] UART4  TX = PC10 -> use NEUQ_WAVE_UART = UART_3\r\n";

    NEUQ_Uart_init(UART_0, NEUQ_WAVE_BAUD);
    NEUQ_Uart_init(UART_3, NEUQ_WAVE_BAUD);

    Wave_WriteTo(USART1, (const uint8_t *)msg_pa9,  sizeof(msg_pa9)  - 1);
    Wave_WriteTo(UART4,  (const uint8_t *)msg_pc10, sizeof(msg_pc10) - 1);
}

uint32_t NEUQ_Wave_GetFrames(void)
{
    return s_frames;
}

// 取某一轴的原始值或滤波值，加速度单位g，角速度单位°/s
float NEUQ_Wave_GetValue(uint8_t axis, uint8_t filtered)
{
    if(axis >= (uint8_t)NEUQ_AXIS_NUM) axis = 0;

    if(axis < (uint8_t)NEUQ_AXIS_GYRO_X)
    {
        return filtered ? g_imu.acc_g[axis] : g_imu.acc_g_raw[axis];
    }
    axis = (uint8_t)(axis - (uint8_t)NEUQ_AXIS_GYRO_X);
    return filtered ? g_imu.gyro_dps[axis] : g_imu.gyro_dps_raw[axis];
}

// 每采样一次调一次，按设定速率成对发原始值和滤波值
void NEUQ_Wave_Tick(void)
{
    float fs    = NEUQ_IMU_GetSampleHz();
    float rate;
    float raw;
    float flt;

    if(g_setting.wave_enable == 0)
    {
        return;                         // 只是不发而已，采集和滤波照常
    }

    rate = (float)NEUQ_Setting_WaveRateHz(g_setting.wave_rate);
    if(rate > fs) rate = fs;
    if(rate <= 0.0f)
    {
        return;
    }

    s_phase += rate;
    if(s_phase < fs)
    {
        return;                         // 这一拍还不到发送时刻
    }
    s_phase -= fs;

    // 两个值取自同一次采样，时间基准、单位、尺度完全一样，可以直接叠着看
    raw = NEUQ_Wave_GetValue(g_setting.wave_axis, 0);
    flt = NEUQ_Wave_GetValue(g_setting.wave_axis, 1);

#if (NEUQ_WAVE_PROTOCOL == NEUQ_WAVE_PROTO_FIREWATER)
    {
        // FireWater文本协议，"ch0,ch1\n"，上位机数据格式要选FireWater
        char     text[40];
        uint8_t  buf[40];
        int      n = snprintf(text, sizeof(text), "%.4f,%.4f\n", raw, flt);

        if(n <= 0)
        {
            return;
        }
        memcpy(buf, text, (uint32_t)n);
        Wave_ReportResult(Wave_WriteBounded(buf, (uint32_t)n));
    }
#else
    {
        // JustFloat二进制协议，两个float加帧尾 00 00 80 7F
        uint8_t buf[12];
        memcpy(&buf[0], &raw, 4);
        memcpy(&buf[4], &flt, 4);
        buf[8]  = 0x00;
        buf[9]  = 0x00;
        buf[10] = 0x80;
        buf[11] = 0x7F;
        Wave_ReportResult(Wave_WriteBounded(buf, 12));
    }
#endif
}
