/* motor_speed — 编码器测速: 快/精双窗滑动平均 + 自适应切换 + 低通滤波 */

#include "motor_speed.h"   /* app_common.h → 库 + config.h */
#include "motor_hw.h"      /* Motor_HW_ReadEncoder */

static volatile int32_t last_enc = 0;
static int32_t enc_ring[FINE_WINDOW_MS];     // 环形缓冲: 最近 FINE_WINDOW_MS 个 1ms 计数差
static uint8_t  enc_ri = 0;                  // 环形缓冲写指针
static int32_t  enc_sum_fine = 0;            // 精窗计数和
static int32_t  enc_sum_fast = 0;            // 快窗计数和
static uint8_t  speed_tick = 0;              // 转速更新节拍计数
static uint8_t  use_fast = 1;                // 当前测速窗口: 1=快窗 0=精窗

/* 初始化: 记录编码器基准计数 */
void Speed_Init(void)
{
    last_enc = Motor_HW_ReadEncoder();
}

/* 1ms 累加编码器计数差入快/精双窗环形缓冲 */
void Speed_Accum1ms(void)
{
    int32_t cnt = Motor_HW_ReadEncoder();
    int32_t delta = cnt - last_enc;
    last_enc = cnt;
    if (delta > 32768)       delta -= 65536;   // 正方向回绕
    else if (delta < -32768) delta += 65536;   // 负方向回绕

    enc_sum_fine += delta - enc_ring[enc_ri];  // 精窗: 加新减最旧
    enc_sum_fast += delta - enc_ring[(enc_ri - FAST_WINDOW_MS + FINE_WINDOW_MS) % FINE_WINDOW_MS];
    // 快窗: 减 FAST_WINDOW_MS 前那项
    enc_ring[enc_ri] = delta;
    if (++enc_ri >= FINE_WINDOW_MS) enc_ri = 0;
}

/* 计算转速: 自适应窗口切换 + 低通滤波 */
float Speed_Calc(float target, float actual)
{
    if (++speed_tick < SPEED_PERIOD_MS) return actual;
    speed_tick = 0;

    // 自适应窗口: 误差大用快窗(响应快), 误差小用精窗(分辨率高→稳态误差小); 迟滞防抖
    float err_now = target - actual;
    if (use_fast) {
        if (err_now > -ADAPT_ERR_FINE && err_now < ADAPT_ERR_FINE) use_fast = 0;
    } else {
        if (err_now > ADAPT_ERR_FAST || err_now < -ADAPT_ERR_FAST) use_fast = 1;
    }
    float rpm = use_fast ? (float)enc_sum_fast * FAST_SCALE : (float)enc_sum_fine * FINE_SCALE;

    return actual + SPEED_LPF_ALPHA * (rpm - actual);
}
