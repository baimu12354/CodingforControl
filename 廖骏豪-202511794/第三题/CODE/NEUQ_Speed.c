/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_Speed
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
********************************************************************************************************************/
#include "NEUQ_Config.h"
#include "NEUQ_Speed.h"

static float   s_cnt_per_rev = 1.0f;    // 输出轴每转计数 = PPR × 4 × 减速比
static float   s_rpm_per_cnt = 1.0f;    // 1 个计数对应多少 RPM
static float   s_rpm_raw     = 0.0f;
static float   s_rpm         = 0.0f;    // 滤波后转速，给 PID 和回传用
static int32_t s_delta       = 0;       // 本次计数增量
static uint8_t s_enc_sel     = 0u;      // 0=还没定 1=TIM4(PB6/PB7) 2=TIM3(PA6/PA7)
static uint8_t s_enc_hit     = 0u;      // 连续有计数的拍数

static volatile float s_dir_sign = NEUQ_ENCODER_DIR_SIGN;   // 测速方向，可用 0x14 命令在线改

// 库里给编码器脚配的是 GPIO_NOPULL，而霍尔编码器多半是开漏输出，
// 没有上拉就一直是低电平，表现就是"电机在转、i1 恒为 0"。这里补内部上拉。
static void NEUQ_Speed_PullUp(GPIO_TypeDef *port, uint16_t pin, uint32_t af)
{
    GPIO_InitTypeDef gi;

    gi.Pin       = pin;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_HIGH;
    gi.Alternate = af;
    HAL_GPIO_Init(port, &gi);
}

void NEUQ_Speed_Init(void)
{
    // 两对编码器脚都初始化：一块驱动板的电机编码器在 PB6/PB7，另一块在 PA6/PA7。
    // 哪一对先出计数就用哪一对，换板子不用改代码。
    NEUQ_Encoder_init(NEUQ_ENCODER_DEV, NEUQ_ENCODER_PHASE_A, NEUQ_ENCODER_PHASE_B);
    NEUQ_Speed_PullUp(NEUQ_ENC_PULLUP_GPIO, NEUQ_ENC_PULLUP_PIN, NEUQ_ENC_PULLUP_AF);

    NEUQ_Encoder_init(Encoder_TIM3, Encoder_TIM3_PhaseA_PA6, Encoder_TIM3_PhaseB_PA7);
    NEUQ_Speed_PullUp(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_AF2_TIM3);

    s_enc_sel = 0u;
    s_enc_hit = 0u;

    s_cnt_per_rev = NEUQ_ENCODER_PPR * NEUQ_ENCODER_MULT * NEUQ_ENCODER_GEAR;
    if (s_cnt_per_rev < 1.0f) { s_cnt_per_rev = 1.0f; }     // 防止配置写错除零

    s_rpm_per_cnt = 60.0f / (s_cnt_per_rev * NEUQ_CONTROL_DT);

    s_rpm_raw = 0.0f;
    s_rpm     = 0.0f;
    s_delta   = 0;
}

// 每 5ms 调一次：读增量 -> 换算 -> 低通。
// 库的读函数以 0x7FFF 为基准、正负表示转向，读完立刻清零，所以每个周期的增量互不叠加。
float NEUQ_Speed_Update(void)
{
    int32_t delta;

    {
        int32_t d4 = NEUQ_Encoder_Read(NEUQ_ENCODER_DEV);   // TIM4 = PB6/PB7
        int32_t d3 = NEUQ_Encoder_Read(Encoder_TIM3);       // TIM3 = PA6/PA7

        NEUQ_Encoder_Clear(NEUQ_ENCODER_DEV);
        NEUQ_Encoder_Clear(Encoder_TIM3);

        if (s_enc_sel == 0u)
        {
            // 连续 3 拍都有计数才认定，避免上电噪声误锁
            if ((d4 != 0) || (d3 != 0))
            {
                if (++s_enc_hit >= 3u) { s_enc_sel = (d4 != 0) ? 1u : 2u; }
            }
            else
            {
                s_enc_hit = 0u;
            }
            delta = 0;
        }
        else if (s_enc_sel == 1u)
        {
            delta = d4;
        }
        else
        {
            delta = d3;
        }
    }

    s_delta   = delta;
    s_rpm_raw = (float)delta * s_rpm_per_cnt * s_dir_sign;

    // 一阶低通，压掉编码器量化台阶（ALPHA = 1.0f 等价于不滤波）
    s_rpm += NEUQ_SPEED_FILTER_ALPHA * (s_rpm_raw - s_rpm);

    return s_rpm;
}

float NEUQ_Speed_Get(void)
{
    return s_rpm;
}

int32_t NEUQ_Speed_GetRawDelta(void)
{
    return s_delta;
}

// 对应串口命令 0x14。写在主循环、读在 5ms 中断，32 位对齐赋值不会出现半新半旧
void NEUQ_Speed_SetDirSign(float sign)
{
    s_dir_sign = (sign < 0.0f) ? (-1.0f) : (1.0f);
}

float NEUQ_Speed_GetDirSign(void)
{
    return s_dir_sign;
}

float NEUQ_Speed_GetRpmPerCount(void)
{
    return s_rpm_per_cnt;
}
