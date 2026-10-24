/* motor_report — 周期上报: 目标/实际/Kp/Ki/Kd/占空比/电压, 逗号分隔文本 */

#include "motor_report.h"   /* app_common.h → 库 + config.h */

static uint32_t last_report = 0;

// 把 float 转成 "%.3f" 文本
static int Ftoa(char *dst, float val)
{
    int len = 0;
    if (val < 0) { dst[len++] = '-'; val = -val; }
    long ipart = (long)val;
    int frac = (int)((val - (float)ipart) * 1000.0f + 0.5f);
    if (frac >= 1000) { ipart += 1; frac -= 1000; }

    char tmp[16];
    int t = 0;
    if (ipart == 0) tmp[t++] = '0';
    else while (ipart) { tmp[t++] = (char)('0' + ipart % 10); ipart /= 10; }
    while (t) dst[len++] = tmp[--t];

    dst[len++] = '.';
    dst[len++] = (char)('0' + (frac / 100) % 10);
    dst[len++] = (char)('0' + (frac / 10) % 10);
    dst[len++] = (char)('0' + frac % 10);
    return len;
}

/* 追加一个逗号分隔的浮点字段 */
static int AppendField(char *dst, float v)
{
    int n = Ftoa(dst, v);
    dst[n] = ',';
    return n + 1;
}

/* 发送一帧逗号分隔的上报数据 */
static void Report_Send(void)
{
    char buf[80];
    int len = 0;

    len += AppendField(buf + len, target_rpm);
    len += AppendField(buf + len, actual_rpm);
    len += AppendField(buf + len, pid.Kp);
    len += AppendField(buf + len, pid.Ki);
    len += AppendField(buf + len, pid.Kd);
    len += AppendField(buf + len, pwm_duty / 100.0f);   /* 带符号占空比 ±100% (±10000=满) */
    len += Ftoa(buf + len, supply_voltage);
    buf[len++] = '\n';                                   /* 供电电压(V) */

    NEUQ_Uart_write_data(COMM_UART, (uint8_t *)buf, len);
}

/* 周期上报(内部按 REPORT_PERIOD_MS 节流) */
void Report_Tick(void)
{
    uint32_t now = time_tick;
    if (now - last_report >= REPORT_PERIOD_MS)
    {
        last_report = now;
        Report_Send();
    }
}
