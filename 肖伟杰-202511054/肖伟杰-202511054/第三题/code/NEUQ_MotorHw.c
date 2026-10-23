#include "NEUQ_MotorHw.h"
#include "NEUQ_MotorRing.h"
#include "NEUQ_MotorApp.h"

/* 库头文件里没写这个原型，函数本身在 NEUQ_Uart.c 里 */
void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

static NEUQ_MotorRxRingType s_rx_ring;
static NEUQ_MotorTxRingType s_tx_ring;

static TIM_HandleTypeDef s_tim5;
static volatile uint32_t s_ms;

/* ============================== 编码器 ============================== */

static void MotorHw_EncZero(void)
{
    NEUQ_Encoder_Clear(NEUQ_MOTORH_ENC_DEV);
}

int32_t NEUQ_MotorHw_EncReadClear(void)
{
    int32_t cnt = NEUQ_Encoder_Read(NEUQ_MOTORH_ENC_DEV);

    NEUQ_Encoder_Clear(NEUQ_MOTORH_ENC_DEV);
    return cnt;
}

/* ============================== 初始化 ============================== */

void NEUQ_MotorHw_Init(void)
{
    /* ① PWM：上电占空比 0，等 START 命令 */
    NEUQ_PWM_init(NEUQ_MOTORH_PWM_PORT, NEUQ_MOTORH_PWM_FREQ, 0u);

    /* ② 方向脚：先给正转电平，保证一个确定状态 */
    NEUQ_GPIO_init(NEUQ_MOTORH_DIR_GPIO, GPIO_Mode_Output_PushPull, NEUQ_MOTORH_DIR_FWD_LEVEL);

    /* ③ 编码器：TIM4 硬件正交编码器 */
    NEUQ_Encoder_init(NEUQ_MOTORH_ENC_DEV, NEUQ_MOTORH_ENC_A, NEUQ_MOTORH_ENC_B);
    MotorHw_EncZero();

    /* 编码器多为开集输出，打开 PB6/PB7 内部上拉；高速时建议外部再并 4.7k 上拉 */
    {
        GPIO_InitTypeDef enc_pu;

        enc_pu.Pin       = (uint32_t)(GPIO_PIN_6 | GPIO_PIN_7);
        enc_pu.Mode      = GPIO_MODE_AF_PP;
        enc_pu.Pull      = GPIO_PULLUP;
        enc_pu.Speed     = GPIO_SPEED_FREQ_HIGH;
        enc_pu.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &enc_pu);
    }

    /* ④ 串口：接收中断只入队，发送由主循环泵驱动 */
    NEUQ_Uart_init(NEUQ_MOTORH_UART, UART_BAUD_115200);
    NEUQ_MotorRxRing_Init(&s_rx_ring);
    NEUQ_MotorTxRing_Init(&s_tx_ring);
    NEUQ_Uart_Enable_RX_Interrupt(NEUQ_MOTORH_UART, (uint8_t)NEUQ_MOTORH_INT_PRIO_UART);

    /* ⑤ TIM5：1 ms 定周期 */
    __HAL_RCC_TIM5_CLK_ENABLE();

    s_tim5.Instance               = TIM5;
    s_tim5.Init.Prescaler         = 84u - 1u;           /* 84 MHz / 84 = 1 MHz */
    s_tim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_tim5.Init.Period            = 1000u - 1u;         /* 1 kHz = 1 ms */
    s_tim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_tim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&s_tim5);

    HAL_NVIC_SetPriority(TIM5_IRQn, (uint32_t)NEUQ_MOTORH_INT_PRIO_CTRL, 0u);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    HAL_TIM_Base_Start_IT(&s_tim5);

    s_ms = 0u;
}

/* ============================== PWM / 方向 ============================== */

void NEUQ_MotorHw_SetOutput(float duty, uint8_t dir_forward)
{
    uint32_t pwm;

    if (duty < 0.0f)
    {
        duty = -duty;
    }
    if (duty >= 10000.0f)
    {
        pwm = 10000u;
    }
    else
    {
        pwm = (uint32_t)duty;
    }

    /* 先定方向，再给占空比 */
    if (dir_forward != 0u)
    {
        NEUQ_GPIO_write(NEUQ_MOTORH_DIR_GPIO, NEUQ_MOTORH_DIR_FWD_LEVEL);
    }
    else
    {
        NEUQ_GPIO_write(NEUQ_MOTORH_DIR_GPIO, NEUQ_MOTORH_DIR_REV_LEVEL);
    }

    NEUQ_PWM_set_duty(NEUQ_MOTORH_PWM_PORT, pwm);
}

/* ============================== 串口接收 ============================== */

void NEUQ_MotorApp_RxByte(uint8_t byte)
{
    (void)NEUQ_MotorRxRing_Push(&s_rx_ring, byte);
}

uint8_t NEUQ_MotorHw_RxPop(uint8_t *byte)
{
    return NEUQ_MotorRxRing_Pop(&s_rx_ring, byte);
}

uint32_t NEUQ_MotorHw_RxDrop(void)
{
    return s_rx_ring.drop;
}

/* ============================== 串口发送 ============================== */

void NEUQ_MotorHw_TxFrame(const uint8_t *buf, uint16_t len)
{
    (void)NEUQ_MotorTxRing_PushFrame(&s_tx_ring, buf, len);
}

void NEUQ_MotorHw_TxPump(void)
{
    uint8_t byte;

    if ((NEUQ_MOTORH_UART_REG->SR & USART_SR_TXE) == 0u)
    {
        return;
    }
    if (NEUQ_MotorTxRing_Pop(&s_tx_ring, &byte) == 0u)
    {
        return;
    }
    NEUQ_MOTORH_UART_REG->DR = (uint16_t)byte;
}

uint32_t NEUQ_MotorHw_TxDrop(void)
{
    return s_tx_ring.drop;
}

uint32_t NEUQ_MotorHw_Millis(void)
{
    return s_ms;
}

/* ============================== TIM5 1 ms 中断 ============================== */

void TIM5_IRQHandler(void);
void TIM5_IRQHandler(void)
{
    __HAL_TIM_CLEAR_IT(&s_tim5, TIM_IT_UPDATE);
    s_ms++;
    NEUQ_MotorApp_Tick(NEUQ_MotorHw_EncReadClear());
}

/* ============================== 串口错误兜底 ============================== */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == NEUQ_MOTORH_UART_REG)
    {
        /* 先读 SR 再读 DR，清掉 ORE/NE/FE/PE */
        (void)NEUQ_MOTORH_UART_REG->SR;
        (void)NEUQ_MOTORH_UART_REG->DR;

        if (huart->pRxBuffPtr != NULL)
        {
            (void)HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, 1u);
        }
    }
}
