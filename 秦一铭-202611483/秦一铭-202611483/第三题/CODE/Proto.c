#include "NEUQ_lib_include.h"

/* ══════════ 上位机 → 板子：字节包 ══════════
   AA  LEN  CMD  DATA  SUM  BB
   LEN = CMD 加 DATA 一共字节数（1~7）
   SUM = (LEN + CMD + ΣDATA) 取低 8 位，帧头帧尾不算
   CMD  01 启动（DATA=目标转速）  02 停止
        03 改目标（运行中可用）  04 改 PID（只能停止时改）
        07 启动（用当前目标，给按钮用）
   DATA 一律 2 字节、高字节在前；PID 填 实际值×100

   滑条帧（没有校验）：
   A5  通道  数值  5A  A5
   通道  5A 目标转速  5B Kp  5C Ki  5D Kd
   数值 4 字节 float
   ═══════════════════════════════════════════════════ */

volatile uint8_t g_run        = 0;              // 上电先停，等启动命令
volatile int16_t g_target_rpm = 0;
volatile int16_t g_actual_rpm = 0;
volatile int16_t g_pid[3]     = {250, 5, 0};    //默认 Kp=2.50  Ki=0.05  Kd=0

// 解析状态机
typedef enum{
    WAIT_HEAD,      // 等 AA
    WAIT_LEN,       // 读 LEN
    WAIT_CMD,       // 读 CMD
    WAIT_DATA,      // 读 DATA
    WAIT_CHECK,     // 读 SUM
    WAIT_TAIL,      // 等 BB 并校验
    WAIT_SPD,       // 滑条帧：读通道
    WAIT_SPDV,      // 滑条帧：读数值，直到 5A A5
} Step;

static Step    step = WAIT_HEAD;
static uint8_t rx_len, rx_cmd, rx_check, rx_n;
static uint8_t rx_data[6];                      // 最长 6 字节（PID）
static uint8_t spd_sel, spd_n, spd_buf[6];      // 滑条帧用
static volatile uint8_t tx_cnt = 0, tx_ok = 0;  // 中断里写、主循环里读

static uint8_t     fifo_buf[128];
static NEUQ_FIFO_t fifo;
volatile uint32_t overflow = 0;     // FIFO 满、丢字节的次数（调试用）

//限幅
static int16_t Limit(int16_t v, int16_t max)
{
    if (v >  max) return  max;
    if (v < -max) return -max;
    return v;
}

/* 取 rx_data 的第 i、i+1 两个字节，拼成一个 16 位整数（高字节在前）
   值 = 高字节 × 256 + 低字节
   (int16_t) 将超过 32767 的数，减掉 65536，按负数解读 */
static int16_t Get16(uint8_t i)
{
    return (int16_t)(rx_data[i] * 256 + rx_data[i + 1]);
}

//校验通过后执行
static void DoCmd(void)
{
    switch (rx_cmd){
    case 0x01:                                  // 启动，同时设目标转速
        g_target_rpm = Limit(Get16(0), 1200);   // 目标上限 ±1200 rpm
        g_run        = 1;
        break;

    case 0x02:                                  // 停止
        g_run = 0;
        break;

    case 0x03:                                  // 运行中改目标
        g_target_rpm = Limit(Get16(0), 1200);
        break;

    case 0x04:                                  // 改 PID：只有停着才接收
        if (g_run == 0){
            g_pid[0] = Limit(Get16(0), 2000);   // Kp，上限 20.00
            g_pid[1] = Limit(Get16(2), 2000);   // Ki
            g_pid[2] = Limit(Get16(4), 2000);   // Kd
        }
        break;

    case 0x07:                                  // 启动，用当前目标
        g_run = 1;
        break;

    default:
        break;
    }
}

// 执行滑条帧：A5 通道 数值 5A A5
static void DoSlide(void)
{
    union { float f; uint8_t b[4]; } u;
    int32_t v;

    if (spd_n != 4) return;                     // 只认 4 字节 float

    u.b[0] = spd_buf[0];
    u.b[1] = spd_buf[1];
    u.b[2] = spd_buf[2];
    u.b[3] = spd_buf[3];
    v = (int32_t)u.f;

    if (v > 3000 || v < -3000) return;          

    switch (spd_sel){
    case 0x5A: g_target_rpm = Limit((int16_t)v, 1200); break;               // 目标转速
    case 0x5B: if (g_run == 0) g_pid[0] = Limit((int16_t)v, 2000); break;   // Kp
    case 0x5C: if (g_run == 0) g_pid[1] = Limit((int16_t)v, 2000); break;   // Ki
    case 0x5D: if (g_run == 0) g_pid[2] = Limit((int16_t)v, 2000); break;   // Kd
    default:   break;
    }
}

/* 一次只处理一个字节，靠 step 把分次到达的字节拼成一帧 */
static void Proto_Analysis(uint8_t dat)
{
    switch (step){
    case WAIT_HEAD:
        if      (dat == 0xAA) step = WAIT_LEN;      // AA 帧
        else if (dat == 0xA5) step = WAIT_SPD;      // 可能是滑条帧
        break;

    case WAIT_LEN:
        rx_len = dat;
        step   = (rx_len >= 1 && rx_len <= 7) ? WAIT_CMD : WAIT_HEAD;
        break;

    case WAIT_CMD:
        rx_cmd = dat;
        rx_n   = 0;
        step   = (rx_len > 1) ? WAIT_DATA : WAIT_CHECK;
        break;

    case WAIT_DATA:
        rx_data[rx_n++] = dat;
        if (rx_n >= rx_len - 1) step = WAIT_CHECK;
        break;

    case WAIT_CHECK:
        rx_check = dat;
        step     = WAIT_TAIL;
        break;

    case WAIT_TAIL:{
        uint8_t i, sum = 0;
        sum += rx_len;                                  // LEN
        sum += rx_cmd;                                  // CMD
        for (i = 0; i < rx_n; i++) sum += rx_data[i];   // DATA
        if (dat == 0xBB && rx_check == sum) DoCmd();    // 校验通过才执行
        step = WAIT_HEAD;
        break;
    }

    case WAIT_SPD:
        if (dat >= 0x5A && dat <= 0x5D){                // 通道对
            spd_sel = dat;
            spd_n   = 0;
            step    = WAIT_SPDV;
        }else{
            step = WAIT_HEAD;                           // 通道不对
        }
        break;

    case WAIT_SPDV:
        spd_buf[spd_n++] = dat;
        if (spd_n >= 2 && spd_buf[spd_n - 2] == 0x5A && spd_buf[spd_n - 1] == 0xA5){
            spd_n -= 2;                                 // 去掉后缀 5A A5
            DoSlide();
            step = WAIT_HEAD;
        }else if (spd_n >= 6){                          // 太长，当误码扔掉
            step = WAIT_HEAD;
        }
        break;

    }
}

// 回传：目标转速、实际转速、Kp、Ki、Kd（五个 float + JustFloat 帧尾） 
static void Send(void)
{
    float   f[5];
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

    f[0] = g_target_rpm;                
    f[1] = g_actual_rpm;
    f[2] = g_pid[0] / 100.0f;           
    f[3] = g_pid[1] / 100.0f;
    f[4] = g_pid[2] / 100.0f;

    NEUQ_Uart_write_data(UART_0, (uint8_t *)f, sizeof(f));   //转换为数据指针
    NEUQ_Uart_write_data(UART_0, tail, 4);
}

void NEUQ_Proto_Init(void)
{
    NEUQ_FIFO_init(&fifo, fifo_buf, sizeof(fifo_buf));
    NEUQ_Uart_init(UART_0, UART_BAUD_115200);   // PA9 发 PA10 收
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 5);   
    NEUQ_PIT_init_ms(PIT_CH1, 1);               // 1ms 
}

void NEUQ_Proto_RxByte(uint8_t dat)             // 串口中断里调用：只入队
{
    if (NEUQ_FIFO_write_byte(&fifo, dat) != NEUQ_FIFO_OK){
        overflow++;                             // FIFO 满了，这个字节丢失
    }
}

void NEUQ_Proto_Tick(void)                      // 1ms 中断里调用
{
    if (++tx_cnt >= 10){                        // 1ms × 10 = 10ms 回传一次
        tx_cnt = 0;
        tx_ok  = 1;
    }
}

void NEUQ_Proto_Task(void)                      // 主循环里调用
{
    uint8_t dat;

    while (NEUQ_FIFO_read_byte(&fifo, &dat) == NEUQ_FIFO_OK){
        Proto_Analysis(dat);                    // 出队 ->状态机
    }

    if (tx_ok){
        tx_ok = 0;
        Send();
    }
}