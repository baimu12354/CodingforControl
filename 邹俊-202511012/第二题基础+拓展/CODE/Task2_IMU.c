/*********************************************************************************************************************
* 文件名称          Task2_IMU
* 功能说明          第二题：六轴采集 / 滤波 / 上传 / Flash 参数 / 姿态解算 / 主函数（实现），说明见 Task2_IMU.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "Task_Config.h"
#include "Task2_IMU.h"

#if (TASK_SELECT == 2)

#include "My_Key.h"
#include "My_Lcd.h"
#include "My_Vofa.h"
#include "Task2_Menu.h"

#define T2_DT_S             ((float)T2_SAMPLE_PERIOD_MS / 1000.0f)  // 采样周期，单位 s
#define T2_PI               (3.14159265f)
#define T2_DEG_TO_RAD       (0.01745329252f)
#define T2_RECORD_MAGIC     (0x31503254UL)      // Flash 记录的"身份标识"，内存里的字节是 'T' '2' 'P' '1'

T2_Imu_Struct      g_t2;
T2_Attitude_Struct g_t2_attitude;
T2_Calib_Struct    g_t2_calib;

static float   s_alpha[T2_AXIS_NUM];        // 六个轴各自的低通滤波系数
static uint8_t s_filter_started = 0;        // 0 表示还没收到第一个样本
static uint8_t s_fail_count = 0;            // IIC 连续读失败次数
static uint8_t s_key_timer = 0;             // 5 ms 中断里数到 2 就扫描一次按键

// 保存在 Flash 里的一条参数记录：身份标识 + 六个截止频率 + 校验和，共 32 字节，整体读写
typedef struct {
    uint32_t magic;                     // 身份标识，用来判断 Flash 里存的是不是我们的参数
    float    cutoff_hz[T2_AXIS_NUM];    // 六个轴的截止频率
    uint32_t checksum;                  // 前面 28 个字节逐个相加的和，用来发现数据损坏
} T2_Record_Struct;

//====================================================== 滤波 ======================================================
// 一阶低通滤波：  滤波后 = 上次滤波后 + α × (本次原始值 - 上次滤波后)
// α 越小越平滑（但变化跟得越慢）。按截止频率 fc 换算：α = dt / (RC + dt)，其中 RC = 1 / (2π·fc)
// 例如 fc = 5 Hz、dt = 5 ms 时 α ≈ 0.136；fc = 1 Hz 时 α ≈ 0.03（很平滑）；fc = 50 Hz 时 α ≈ 0.61（接近原始）
static float t2_alpha_from_cutoff(float cutoff_hz)
{
    float rc = 1.0f / (2.0f * T2_PI * cutoff_hz);
    return T2_DT_S / (rc + T2_DT_S);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     修改某个轴的截止频率，立即生效
// 备注信息     只重算这个轴的 α，不清空滤波器的历史值，所以波形不会突然跳变；
//              α 是一个 32 位 float，中断读它时一次读完，不会读到"改了一半"的值
//-------------------------------------------------------------------------------------------------------------------
void T2_Imu_set_cutoff(uint8_t axis, float cutoff_hz)
{
    if (axis >= T2_AXIS_NUM) {
        return;
    }
    if (cutoff_hz < T2_CUTOFF_MIN_HZ) cutoff_hz = T2_CUTOFF_MIN_HZ;
    if (cutoff_hz > T2_CUTOFF_MAX_HZ) cutoff_hz = T2_CUTOFF_MAX_HZ;
    g_t2.cutoff_hz[axis] = cutoff_hz;
    s_alpha[axis]        = t2_alpha_from_cutoff(cutoff_hz);
    g_t2.param_dirty     = 1;
    g_t2.save_failed     = 0;
}

//=================================================== Flash 参数 ===================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算校验和：把记录前 28 个字节（身份标识 + 六个截止频率）逐个相加
// 备注信息     保存时把校验和一起写进去；读出来时重新算一遍，对不上就说明数据坏了（或者根本没保存过）
//-------------------------------------------------------------------------------------------------------------------
static uint32_t t2_checksum(const T2_Record_Struct *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t       sum   = 0;
    uint32_t       i;

    for (i = 0; i < sizeof(T2_Record_Struct) - sizeof(record->checksum); i++) {
        sum += bytes[i];
    }
    return sum;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     上电读取 Flash 里保存的截止频率；读不到有效记录就用默认值
// 备注信息     有效记录必须同时满足：身份标识正确、校验和正确、每个数都在允许范围内。
//              全新的 Flash 读出来全是 0xFF，身份标识对不上，自然会使用默认值
//-------------------------------------------------------------------------------------------------------------------
static void t2_load_param(void)
{
    T2_Record_Struct record;
    uint8_t          valid = 0;
    uint8_t          i;

    for (i = 0; i < T2_AXIS_NUM; i++) {
        g_t2.cutoff_hz[i] = (i < 3U) ? T2_DEFAULT_ACC_CUTOFF_HZ : T2_DEFAULT_GYRO_CUTOFF_HZ;
    }

    g_t2.flash_ok = (NEUQ_Flash_init() == NEUQ_FLASH_OK) ? 1U : 0U;
    if (g_t2.flash_ok &&
        NEUQ_Flash_read_sector(T2_FLASH_SECTOR, (uint8_t *)&record, sizeof(record)) == NEUQ_FLASH_OK &&
        record.magic == T2_RECORD_MAGIC &&
        record.checksum == t2_checksum(&record)) {
        valid = 1;
        for (i = 0; i < T2_AXIS_NUM; i++) {
            if (!(record.cutoff_hz[i] >= T2_CUTOFF_MIN_HZ && record.cutoff_hz[i] <= T2_CUTOFF_MAX_HZ)) {
                valid = 0;              // 写成 !(a && b) 的形式：NaN 参与比较结果为假，也会被判为无效
            }
        }
    }
    if (valid) {
        for (i = 0; i < T2_AXIS_NUM; i++) {
            g_t2.cutoff_hz[i] = record.cutoff_hz[i];
        }
    }
    for (i = 0; i < T2_AXIS_NUM; i++) {
        s_alpha[i] = t2_alpha_from_cutoff(g_t2.cutoff_hz[i]);
    }
    g_t2.flash_has_record = valid;
    g_t2.param_dirty      = 0;
    g_t2.save_failed      = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把当前六个截止频率保存到 Flash
// 返回参数     1 成功  0 失败
// 备注信息     流程：组记录 → 擦除扇区并写入（库函数 NEUQ_Flash_write_sector）→ 读回来逐字节比较。
//              在主循环里执行，耗时几十毫秒，这期间采样中断照常运行，采集和滤波不会停
//-------------------------------------------------------------------------------------------------------------------
uint8_t T2_Imu_save_param(void)
{
    T2_Record_Struct record;
    T2_Record_Struct check;
    uint8_t          i;

    if (!g_t2.flash_ok) {
        g_t2.save_failed = 1;
        return 0;
    }
    record.magic = T2_RECORD_MAGIC;
    for (i = 0; i < T2_AXIS_NUM; i++) {
        record.cutoff_hz[i] = g_t2.cutoff_hz[i];
    }
    record.checksum = t2_checksum(&record);

    if (NEUQ_Flash_write_sector(T2_FLASH_SECTOR, (const uint8_t *)&record, sizeof(record)) == NEUQ_FLASH_OK &&
        NEUQ_Flash_read_sector(T2_FLASH_SECTOR, (uint8_t *)&check, sizeof(check)) == NEUQ_FLASH_OK &&
        memcmp(&record, &check, sizeof(record)) == 0) {
        g_t2.flash_has_record = 1;
        g_t2.param_dirty      = 0;
        g_t2.save_failed      = 0;
        return 1;
    }
    g_t2.save_failed = 1;
    return 0;
}

//====================================================== 采样 ======================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读一次 MPU6050：从寄存器 0x3B 开始连续读 14 个字节
// 参数说明     raw         输出六个轴的原始计数，顺序 AX AY AZ GX GY GZ
// 参数说明     temp_raw    输出温度原始值
// 返回参数     1 成功  0 IIC 读失败
// 备注信息     14 个字节依次是：加速度 X Y Z（各 2 字节）、温度（2 字节）、角速度 X Y Z（各 2 字节），每个数高字节在前。
//              库函数 NEUQ_MPU6050_read_data() 不告诉我们 IIC 有没有读成功，所以这里直接调用库的 IIC 读函数，检查返回值
//-------------------------------------------------------------------------------------------------------------------
static uint8_t t2_read_sensor(int16_t raw[6], int16_t *temp_raw)
{
    uint8_t buf[14];

    if (!NEUQ_Hard_IIC_read_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14)) {
        return 0;
    }
    raw[0]    = (int16_t)((buf[0]  << 8) | buf[1]);     // 加速度 X
    raw[1]    = (int16_t)((buf[2]  << 8) | buf[3]);     // 加速度 Y
    raw[2]    = (int16_t)((buf[4]  << 8) | buf[5]);     // 加速度 Z
    *temp_raw = (int16_t)((buf[6]  << 8) | buf[7]);     // 温度
    raw[3]    = (int16_t)((buf[8]  << 8) | buf[9]);     // 角速度 X
    raw[4]    = (int16_t)((buf[10] << 8) | buf[11]);    // 角速度 Y
    raw[5]    = (int16_t)((buf[12] << 8) | buf[13]);    // 角速度 Z
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理一次采样：换算 → 六轴滤波 → 单轴上传 → 零偏校准 / 姿态更新
//-------------------------------------------------------------------------------------------------------------------
static void t2_process_sample(const int16_t raw[6], int16_t temp_raw)
{
    float   gyro_dps[3];
    uint8_t i;

    // 1. 原始计数换算成物理量（库函数，加速度 4096 计数 = 1 g，角速度 16.4 计数 = 1 dps），然后每个轴各自低通滤波
    for (i = 0; i < T2_AXIS_NUM; i++) {
        g_t2.raw[i] = raw[i];
        if (i < 3U) {
            g_t2.value[i] = MPU6050_acc_transition(raw[i]);
        }
        else {
            g_t2.value[i] = MPU6050_gyro_transition(raw[i]);
        }
        if (s_filter_started) {
            g_t2.filtered[i] += s_alpha[i] * (g_t2.value[i] - g_t2.filtered[i]);
        }
        else {
            g_t2.filtered[i] = g_t2.value[i];       // 第一个样本直接作为输出，上电时不会从 0 慢慢爬升
        }
    }
    s_filter_started   = 1;
    g_t2.temperature   = (float)temp_raw / 340.0f + 36.53f;    // MPU6050 手册给出的温度换算公式
    g_t2.sample_count++;

    // 2. 上传：只发选中的一个轴，原始值和滤波值来自同一次采样、单位相同，放在同一帧里
    //    VOFA+ 中 I0 = 原始，I1 = 滤波
    if (g_t2.upload) {
        uint8_t axis = g_t2.axis;
        float   channel[2];
        channel[0] = g_t2.value[axis];
        channel[1] = g_t2.filtered[axis];
        My_Vofa_push(channel, 2);
    }

    // 3. 姿态：用原始数据（不用可调低通后的数据），这样改滤波参数不会影响姿态结果
    for (i = 0; i < 3U; i++) {
        gyro_dps[i] = g_t2.value[3U + i];
    }
    if (g_t2.calib_request) {
        g_t2.calib_request = 0;
        T2_Calib_start(&g_t2_calib);
    }
    if (g_t2_calib.state == T2_CALIB_RUNNING) {
        // 校准中：只采集不解算；校准完成时用平均加速度算出初始姿态
        if (T2_Calib_feed(&g_t2_calib, gyro_dps, &g_t2.value[0]) == T2_CALIB_DONE) {
            T2_Attitude_reset(&g_t2_attitude, g_t2_calib.accel_mean[0], g_t2_calib.accel_mean[1], g_t2_calib.accel_mean[2]);
        }
    }
    else if (g_t2_calib.state == T2_CALIB_DONE) {
        if (g_t2.zero_yaw_request) {
            g_t2.zero_yaw_request = 0;
            T2_Attitude_zero_yaw(&g_t2_attitude);
        }
        // 角速度先减去零偏；剩下小于 0.1 dps 的部分当作 0（静置时的残余零偏和噪声不再积分，航向不会慢慢漂）
        for (i = 0; i < 3U; i++) {
            gyro_dps[i] -= g_t2_calib.gyro_bias[i];
            if (gyro_dps[i] > -T2_GYRO_DEADBAND_DPS && gyro_dps[i] < T2_GYRO_DEADBAND_DPS) {
                gyro_dps[i] = 0.0f;
            }
        }
        T2_Attitude_update(&g_t2_attitude,
                           gyro_dps[0] * T2_DEG_TO_RAD, gyro_dps[1] * T2_DEG_TO_RAD, gyro_dps[2] * T2_DEG_TO_RAD,
                           g_t2.value[0], g_t2.value[1], g_t2.value[2], T2_DT_S);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     5 ms 定时中断：采样 + 每 10 ms 扫描按键
// 备注信息     IIC 连续 3 次读失败就认为传感器掉线，之后中断里不再访问 IIC（免得每次都等超时），
//              由主循环每秒尝试重新初始化
//-------------------------------------------------------------------------------------------------------------------
void Task2_PIT_Handler(void)
{
    int16_t raw[6];
    int16_t temp_raw;

    if (g_t2.sensor_ok) {
        if (t2_read_sensor(raw, &temp_raw)) {
            s_fail_count = 0;
            t2_process_sample(raw, temp_raw);
        }
        else {
            s_fail_count++;
            if (s_fail_count >= 3U) {
                g_t2.sensor_ok = 0;
            }
        }
    }

    s_key_timer++;
    if (s_key_timer >= KEY_SCAN_PERIOD_MS / T2_SAMPLE_PERIOD_MS) {
        s_key_timer = 0;
        My_Key_scan();
    }
}

//===================================================== 主函数 =====================================================
static uint8_t t2_init_sensor(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();       // 库里的 IIC 总线复位会先把 PA8/PC9 当普通 IO 操作，要先打开端口时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();
    return NEUQ_MPU6050_init();         // 成功返回 1
}

void Task2_Main(void)
{
    uint32_t last_ui_tick;
    uint32_t last_retry_tick;

    NEUQ_Debug_init();                  // UART_0 115200：调试信息 + VOFA+ 波形共用
    NEUQ_Delay_init();
    My_Key_init();
    My_Vofa_init(T2_VOFA_UART);
    My_Lcd_init();

    // 启动画面：依次显示 Flash、参数、传感器的检测结果
    My_Lcd_print(0, UI_STYLE_TITLE, " Task2  MPU6050  booting...");
    t2_load_param();
    My_Lcd_print(2, (UI_Style_Enum)(g_t2.flash_ok ? UI_STYLE_GOOD : UI_STYLE_WARN), "Flash W25Q16 : %s", g_t2.flash_ok ? "OK" : "NOT FOUND");
    My_Lcd_print(3, UI_STYLE_NORMAL, "Filter param : %s", g_t2.flash_has_record ? "loaded" : "default");
    g_t2.sensor_ok     = t2_init_sensor();
    g_t2.calib_request = g_t2.sensor_ok;    // 传感器正常就在上电时自动做一次零偏校准
    My_Lcd_print(4, (UI_Style_Enum)(g_t2.sensor_ok ? UI_STYLE_GOOD : UI_STYLE_WARN), "MPU6050      : %s", g_t2.sensor_ok ? "OK" : "NOT FOUND");
    My_Lcd_print(5, UI_STYLE_WARN, "Gyro calib 2s: keep still!");

    // 中断优先级：数值越小越优先。SysTick 设为 0（最高），采样中断设为 2。
    // 原因：采样中断里调用的 HAL IIC 函数用 HAL_GetTick() 判断超时，如果 SysTick 优先级更低，
    // 它在采样中断里就不会走动，IIC 总线一旦出问题就会永远等下去
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    NEUQ_PIT_init_ms(PIT_CH1, T2_SAMPLE_PERIOD_MS);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);

    NEUQ_Delay_ms(800);                 // 让启动信息停留一下（此时采样和校准已经在中断里运行）
    My_Key_clear_events();
    Task2_Menu_init();
    Task2_Menu_draw();

    last_ui_tick    = HAL_GetTick();
    last_retry_tick = last_ui_tick;
    while (1) {
        Key_Event_Struct event;
        uint8_t          redraw = 0;
        uint32_t         now;

        // 1. 按键 → 菜单
        while (My_Key_get_event(&event)) {
            redraw |= Task2_Menu_handle_key(&event);
        }
        // 2. 把中断放进缓冲的波形数据发出去
        My_Vofa_poll();
        // 3. 刷新屏幕：有按键立即刷新，否则每 100 ms 刷新一次
        now = HAL_GetTick();
        if (redraw || now - last_ui_tick >= 100U) {
            last_ui_tick = now;
            Task2_Menu_draw();
        }
        // 4. 菜单里请求的耗时操作（保存 Flash）放在这里做，做完再刷新一次屏幕
        if (Task2_Menu_poll()) {
            Task2_Menu_draw();
        }
        // 5. 传感器掉线后每 1 s 尝试重连；重连期间中断里不会访问 IIC（sensor_ok 为 0）
        if (!g_t2.sensor_ok && now - last_retry_tick >= 1000U) {
            last_retry_tick = now;
            if (t2_init_sensor()) {
                s_fail_count = 0;
                if (g_t2_calib.state != T2_CALIB_DONE) {
                    g_t2.calib_request = 1;
                }
                g_t2.sensor_ok = 1;
            }
        }
    }
}

#endif
