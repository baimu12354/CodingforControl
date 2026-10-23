/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对它作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_Main
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VET6
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
* 2026-10-11        考生        考核二（产品版）：两级菜单 + 六轴显示 + 波形滤波设置(Flash掉电保存)
*                              + 四元数姿态解算。应用层要点：
*                              6) 按键四态状态机（空闲->消抖->按下->释放），一次按下只产生一次事件，
*                                 满足任务书“按键须消抖”“长按不反复触发”要求；
*                              7) ffmt()/ufmt() 手动拆分整数与小数拼接字符串并定宽补空格，
*                                 LCD 不使用 sprintf("%.3f")，规避 MicroLIB 浮点格式化花屏残影；
*                              8) Flash保存直接调用库接口（擦除+写入+读回校验），保存期间后台采样照常；
*                              9) LCD采用“进页绘静态Frame一次 + 100ms周期/按键事件全量刷新”，
*                                 库显示函数自带背景填充，无闪烁、实现简单；
*                              后台采集/滤波/标定/解算与数据快照接口均在 NEUQ_Interrupts.c 中实现。
********************************************************************************************************************/
#include "../code/NEUQ_lib/NEUQ_lib_include.h"   // 统一包含 NEUQ 开源库全部头文件
#include <stdio.h>                                  // 仅调试串口 printf 使用（整数/%s，不含浮点格式化）
#include <string.h>
//===========================================================//
//===== 硬件接线（与考核一一致） =====
//===== 按键：上=PE7，下=PE8，确认=PE9，返回=PE10（低电平按下，内部上拉，按下沿消抖） =====
//===== MPU6050：硬件I2C3，SCL=PA8，SDA=PC9，地址 0x68（库内已左移），±8g/±500dps =====
//===== LCD：库内部 SPI3，PB3/PB4/PB5/PD5/PD3/PE2；外置Flash：库内部 SPI1，PA5/PA6/PA7/PB0 =====
//===== VOFA+：调试串口 UART_0(PA9 TX/PA10 RX)，115200，FireWater 两通道：原始值,滤波值 =====
//===========================================================//
#define KEY_UP_PIN      PE7
#define KEY_DOWN_PIN    PE8
#define KEY_OK_PIN      PE9
#define KEY_BACK_PIN    PE10
#define KEY_NUM         4u
/* 按键索引（与 key_pin[] 顺序一致） */
enum { KEY_UP = 0, KEY_DOWN, KEY_OK, KEY_BACK };
#define KEY_SCAN_MS         5u      // 按键状态机扫描周期
#define KEY_DEBOUNCE_SCANS  4u      // 4次扫描确认 = 20ms 消抖（按下/释放各确认一次）
#define LCD_REFRESH_MS      100u    // LCD周期刷新节拍
#define BOOT_HOLD_MS        800u    // 开机画面停留时间（非阻塞，标定与按键扫描同时进行）
#define PIPELINE_MAX_LOOP   4u      // 主循环每轮最多处理的采样帧（VOFA背压，保证UI响应）
//===========================================================//
//===== Flash 参数区（W25Q16，扇区4KB；扇区510远离库可能使用的低地址区） =====
//===========================================================//
#define PARAM_SECTOR        510u
#define PARAM_ADDR          (PARAM_SECTOR * 4096u)
#define PARAM_MAGIC         0x50415231u   // "PAR1"
#define ALPHA_STEP          0.005f
#define ALPHA_MIN           0.010f
#define ALPHA_MAX           1.000f
#define ALPHA_DEFAULT       0.150f
//===========================================================//
//===== 页面与菜单状态 =====
//===========================================================//
typedef enum
{
    PAGE_BOOT = 0,
    PAGE_MAIN,
    PAGE_DATA,
    PAGE_FILTER,
    PAGE_ATT,
} Page_Id_e;
typedef enum
{
    MAIN_DATA = 0,
    MAIN_FILTER,
    MAIN_ATT,
    MAIN_ITEM_NUM,
} Main_Item_e;
typedef enum
{
    FLASH_ST_IDLE = 0,
    FLASH_ST_BUSY,
    FLASH_ST_OK,
    FLASH_ST_FAIL,
} Flash_Status_e;
#define FLASH_RESULT_MS   2000u                 //存完Flash后，“Saved OK”的字样在屏幕上停留 2000 毫秒（2秒）
static Page_Id_e      g_page = PAGE_BOOT;        //上电的时候先是pageboot后面是pagemain
static uint8_t        g_cursor = MAIN_DATA;       // 主菜单光标
static uint8_t        g_flt_cursor = 0;           // 滤波设置页光标 0轴/1发送/2系数/3保存/4返回
static uint8_t        g_edit = 0;                 // 滤波页编辑态：0未编辑，1编辑轴，2编辑alpha
static volatile uint8_t g_need_refresh = 1;      // 按键事件触发的立即刷新标志
static uint32_t       g_ref_tick = 0;           //
static uint32_t       g_key_tick = 0;           //
static uint8_t        g_flash_ok = 0;           //芯片初始化成功了吗
static Flash_Status_e g_flash_st = FLASH_ST_IDLE;
static uint32_t       g_flash_tick = 0;
static uint8_t        g_boot_active = 1;
static uint32_t       g_boot_tick = 0;
static uint32_t       g_boot_ref_tick = 0;
static IMU_Snapshot_t g_snap;                    // 应用层持有的数据快照（只读副本）
static uint8_t        g_calib_done = 0;              // 开机阻塞标定是否已完成（开机页显示用）
/* 串口接收FIFO：NEUQ_Interrupts.c 中 UART0 接收中断 extern 引用并逐字节写入；
 * 本工程暂不解析串口指令，仅缓存接收字节，供后续任务扩展 */
static uint8_t        g_rx_fifo_mem[128];
NEUQ_FIFO_t          g_rx_fifo;
//===========================================================//
//===== 一、按键：标准四态状态机 空闲 -> 消抖 -> 按下 -> 释放 =====
//===== 一次按下（含任意时长长按）只产生一次“按下事件”，无连发、无长按计时 =====
//===========================================================//
typedef enum
{
    KEY_ST_IDLE = 0,     // 空闲：等待电平变低
    KEY_ST_DEBOUNCE,     // 按下消抖：连续4次扫描为低才确认
    KEY_ST_PRESSED,      // 按下已确认：事件锁存一次，等待释放
    KEY_ST_RELEASE,      // 释放消抖：连续4次扫描为高才回空闲
} Key_State_e;
static Key_State_e key_state[KEY_NUM];
static uint8_t     key_cnt[KEY_NUM];
static uint8_t     key_event[KEY_NUM];
static const GPIO_Ports_ENUM key_pin[KEY_NUM] = { KEY_UP_PIN, KEY_DOWN_PIN, KEY_OK_PIN, KEY_BACK_PIN };
static void Key_Init(void)
{
    uint8_t i;
    for (i = 0; i < KEY_NUM; i++)
    {
        NEUQ_GPIO_init(key_pin[i], GPIO_Mode_Input_PullUp, GPIO_High);
        key_state[i] = KEY_ST_IDLE;
        key_cnt[i] = 0;
        key_event[i] = 0;
    }
}
static void Key_FlushAll(void)
{
    uint8_t i;
    for (i = 0; i < KEY_NUM; i++)
    {
        key_state[i] = KEY_ST_IDLE;
        key_cnt[i] = 0;
        key_event[i] = 0;
    }
}
/* 每5ms调用一次的状态机迁移，电平只在迁移时采样 */
static void Key_Scan(void)
{
    uint8_t i;
    uint8_t pressed;
    for (i = 0; i < KEY_NUM; i++)
    {
        pressed = (NEUQ_GPIO_read(key_pin[i]) == GPIO_Low) ? 1u : 0u;
        switch (key_state[i])
        {
            case KEY_ST_IDLE:
                if (pressed)
                {
                    key_state[i] = KEY_ST_DEBOUNCE;
                    key_cnt[i] = 0;
                }
                break;
            case KEY_ST_DEBOUNCE:
                if (pressed)
                {
                    key_cnt[i]++;
                    if (key_cnt[i] >= KEY_DEBOUNCE_SCANS)
                    {
                        key_state[i] = KEY_ST_PRESSED;
                        key_event[i] = 1;        // 确认按下：锁存且仅锁存一次事件
                    }
                }
                else
                {
                    key_state[i] = KEY_ST_IDLE; // 抖动：重新等待
                    key_cnt[i] = 0;
                }
                break;
            case KEY_ST_PRESSED:
                if (!pressed)
                {
                    key_state[i] = KEY_ST_RELEASE;
                    key_cnt[i] = 0;
                }
                break;
            case KEY_ST_RELEASE:
                if (!pressed)
                {
                    key_cnt[i]++;
                    if (key_cnt[i] >= KEY_DEBOUNCE_SCANS)
                    {
                        key_state[i] = KEY_ST_IDLE;   // 释放确认，回到空闲，等待下一次按下
                        key_cnt[i] = 0;
                    }
                }
                else
                {
                    key_state[i] = KEY_ST_PRESSED;   // 释放抖动，仍视为按住
                    key_cnt[i] = 0;
                }
                break;
            default:
                key_state[i] = KEY_ST_IDLE;
                key_cnt[i] = 0;
                key_event[i] = 0;
                break;
        }
    }
}
/* 取走一次按下事件（读后清零），长按不放不会再次返回1 */
static uint8_t Key_Take(uint8_t k)
{
    if (k >= KEY_NUM) return 0;
    if (key_event[k])
    {
        key_event[k] = 0;
        return 1;
    }
    return 0;
}
//===========================================================//
//===== 二、ffmt/ufmt：手动浮点/整数转字符串，定宽右对齐补空格 =====
//===== 替代 sprintf("%.3f")，规避 MicroLIB 下浮点格式化的栈开销与LCD花屏残影 =====
//===========================================================//
/* 无符号整数转字符串；width>数字位数时左侧补空格，width=0 表示自然宽度 */
static void ufmt(char *dst, uint32_t v, uint8_t width)
{
    char tmp[12];
    uint8_t n = 0, i, pad;
    do
    {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v);
    if (n >= width || width == 0u)
    {
        for (i = 0; i < n; i++) dst[i] = tmp[n - 1u - i];
        dst[n] = '\0';
    }
    else
    {
        pad = width - n;
        for (i = 0; i < pad; i++) dst[i] = ' ';
        for (i = 0; i < n; i++) dst[pad + i] = tmp[n - 1u - i];
        dst[width] = '\0';
    }
}
/* 浮点转字符串：四舍五入到 dec 位小数，整数/小数手工拆分拼接；
 * width>实际宽度时左侧补空格，width=0 自然宽度。 */
static void ffmt(char *dst, float value, uint8_t width, uint8_t dec)
{
    char tmp[24];
    char dig[12];
    uint8_t n = 0, dn = 0, i, neg = 0, pad;
    uint32_t scale = 1u, frac, uipart;
    int32_t scaled, ipart;
    float av;
    if (value != value)                       // NaN
    {
        strcpy(dst, "NaN");
        return;
    }
    if (value > 999999.0f) value = 999999.0f; // 显示范围保护
    if (value < -999999.0f) value = -999999.0f;
    if (value < 0.0f) { neg = 1; av = -value; } else { av = value; }
    for (i = 0; i < dec; i++) scale *= 10u;
    scaled = (int32_t)(av * (float)scale + 0.5f);   // 四舍五入到末位（进位自然进整数部分）
    ipart = scaled / (int32_t)scale;
    frac  = (uint32_t)(scaled % (int32_t)scale);
    uipart = (uint32_t)ipart;
    if (neg) tmp[n++] = '-';
    do
    {
        dig[dn++] = (char)('0' + (uipart % 10u));
        uipart /= 10u;
    } while (uipart);
    while (dn > 0u) tmp[n++] = dig[--dn];
    if (dec > 0u)
    {
        tmp[n++] = '.';
        for (i = dec; i > 0u; i--)            // 小数部分定长，高位补0
        {
            uint32_t d = frac;
            uint8_t k;
            for (k = 1u; k < i; k++) d /= 10u;
            tmp[n++] = (char)('0' + (d % 10u));
        }
    }
    tmp[n] = '\0';
    if (width > n)
    {
        pad = width - n;
        for (i = 0; i < pad; i++) dst[i] = ' ';
        strcpy(dst + pad, tmp);
    }
    else
    {
        strcpy(dst, tmp);
    }
}
/* 文本定宽：右侧补空格（左对齐），用于固定LCD单元格清除残影 */
static void Str_PadRight(char *dst, const char *src, uint8_t width)
{
    uint8_t i = 0;
    while (i < width && src[i] != '\0') { dst[i] = src[i]; i++; }
    while (i < width) { dst[i] = ' '; i++; }
    dst[width] = '\0';
}
//===========================================================//
//===== 三、LCD显示：进页绘制静态Frame一次，动态区按100ms周期/按键事件全量刷新 =====
//===== 库显示函数按字符格绘制并自带背景填充，整格覆盖无残影、无闪烁 =====
//===========================================================//
static void Lcd_Text(uint16_t x, uint16_t y, const char *s, uint16_t w)
{
    char buf[42];
    Str_PadRight(buf, s, (uint8_t)((w > 40u) ? 40u : w));
    NEUQ_LCD200_DisplayString(x, y, buf);   // 库函数按字符格绘制，自带背景填充，整格覆盖不残影
}
//===========================================================//
//===== 四、Flash 参数：读取 / 保存（保存时做总线资源隔离） =====
//===========================================================//
typedef struct
{
    uint32_t magic;
    float    alpha;
    uint8_t  axis;
    uint8_t  rsv[3];
} Param_Record_t;
/* 前向声明：保存前立即刷新一次“Saving...”状态行 */
void Page_Refresh(Page_Id_e page);
static void Param_Load(void)
{
    Param_Record_t rec;
    uint8_t buf[12];
    uint8_t i;
    for (i = 0; i < sizeof(buf); i++) buf[i] = 0xFFu;
    if (g_flash_ok && NEUQ_Flash_read(PARAM_ADDR, buf, sizeof(buf)) == NEUQ_FLASH_OK)
    {
        memcpy(&rec, buf, sizeof(rec));
        if (rec.magic == PARAM_MAGIC)
        {
            IMU_Set_Alpha(rec.alpha);
            IMU_Set_UploadAxis(rec.axis);
            printf("Param loaded from Flash.\r\n");
            return;
        }
    }
    IMU_Set_Alpha(ALPHA_DEFAULT);   // 无有效记录：默认系数、默认轴，上传开关始终默认关闭
    IMU_Set_UploadAxis(0);
    printf("No valid param, use default.\r\n");
}
static void Param_SaveNow(void)
{
    Param_Record_t rec;
    uint8_t buf[12];
    uint8_t ok = 1;
    IMU_Snapshot_t s;
    char ab[12];
    IMU_GetSnapshot(&s);
    g_flash_st = FLASH_ST_BUSY;
    g_need_refresh = 1;
    Page_Refresh(g_page);      // 立即显示“Saving...”，随后进入阻塞擦写
    /* 外置Flash走SPI1，PIT采样中断走I2C3、LCD走SPI3，三个外设总线互不占用，
     * 保存期间后台采样照常进行（64帧FIFO缓冲足够覆盖扇区擦写时间），无需暂停任何中断 */
    rec.magic = PARAM_MAGIC;
    rec.alpha = s.filter_alpha;
    rec.axis = s.upload_axis;
    rec.rsv[0] = rec.rsv[1] = rec.rsv[2] = 0xFFu;
    memcpy(buf, &rec, sizeof(rec));
    if (NEUQ_Flash_erase_sector(PARAM_SECTOR) != NEUQ_FLASH_OK) ok = 0;
    if (ok && NEUQ_Flash_write(PARAM_ADDR, buf, sizeof(buf)) != NEUQ_FLASH_OK) ok = 0;
    if (ok)
    {
        uint8_t rd[12];
        uint8_t k;
        for (k = 0; k < sizeof(rd); k++) rd[k] = 0;
        if (NEUQ_Flash_read(PARAM_ADDR, rd, sizeof(rd)) != NEUQ_FLASH_OK ||
            memcmp(rd, buf, sizeof(buf)) != 0)
        {
            ok = 0;
        }
    }
    g_flash_st = ok ? FLASH_ST_OK : FLASH_ST_FAIL;
    g_flash_tick = HAL_GetTick();
    g_need_refresh = 1;
    ffmt(ab, s.filter_alpha, 0, 3);
    //printf("Save %s: alpha=%s axis=%u\r\n", ok ? "OK" : "FAIL", ab, (unsigned)s.upload_axis);
}
//===========================================================//
//===== 五、VOFA+ FireWater：同一帧的 原始值,滤波值（两值严格同一次采样、同时基同量纲） =====
//===========================================================//
static void VOFA_SendPair(const IMU_Snapshot_t *s)
{
    char b1[14], b2[14], line[36];
    float raw_v, flt_v;
    if (!s->upload_enable) return;
    if (s->upload_axis < 3u)
    {
        raw_v = s->acc_g[s->upload_axis];
        flt_v = s->acc_g_flt[s->upload_axis];
    }
    else
    {
        uint8_t k = s->upload_axis - 3u;
        raw_v = s->gyro_dps[k];
        flt_v = s->gyro_dps_flt[k];
    }
    ffmt(b1, raw_v, 0, 4);
    ffmt(b2, flt_v, 0, 4);
    strcpy(line, b1);
    strcat(line, ",");
    strcat(line, b2);
    strcat(line, "\n");
    NEUQ_Uart_write_data(UART_0, (uint8_t *)line, (uint32_t)strlen(line));
}
//===========================================================//
//===== 六、页面绘制（Frame只画静态内容一次；Refresh按脏标记局部刷新） =====
//===========================================================//
static void Page_Enter(Page_Id_e page);
static const char *Axis_Name(uint8_t axis)
{
    static const char *name[6] = { "AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ" };
    return name[axis];
}
//---------------- 开机页（标定进度实时显示，非阻塞） ----------------
static void Page_Boot_Draw(void)
{
    NEUQ_LCD200_SetBackColor(LCD_BLACK);
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    NEUQ_LCD200_SetPenColor(LCD_CYAN);
    Lcd_Text(20, 60, "MPU6050 Task2", 17);
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    Lcd_Text(20, 100, "6-axis filter + AHRS", 22);
    Lcd_Text(20, 240, "Keys: Up Down OK Back", 22);
}
static void Page_Boot_Refresh(void)
{
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    if (!g_snap.mpu_ok)
    {
        NEUQ_LCD200_SetPenColor(LCD_RED);
        Lcd_Text(20, 150, "MPU6050 NOT FOUND!", 20);
    }
    else
    {
        NEUQ_LCD200_SetPenColor(LCD_GREEN);
        Lcd_Text(20, 150, "MPU6050 OK", 20);
        if (g_calib_done)
        {
            Lcd_Text(20, 190, "Calib DONE", 22);
        }
        else
        {
            NEUQ_LCD200_SetPenColor(LCD_YELLOW);
            Lcd_Text(20, 190, "Calibrating... keep still", 22);
        }
    }
}
//---------------- 主菜单页 ----------------
static void Page_Main_Frame(void)
{
    NEUQ_LCD200_SetBackColor(LCD_BLACK);
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    NEUQ_LCD200_SetPenColor(LCD_CYAN);
    Lcd_Text(10, 8, "MPU6050 MENU", 12);
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    Lcd_Text(28, 60,  "1. Six-axis data", 18);
    Lcd_Text(28, 100, "2. Wave & Filter", 18);
    Lcd_Text(28, 140, "3. Attitude AHRS", 18);
    Lcd_Text(4, 224, "Up/Dn move  OK enter", 26);
}
static void Page_Main_Refresh(void)
{
    char line[42], num[12], b[12];
    /* 光标 */
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_GREEN);
    Lcd_Text(4, 60,  (g_cursor == MAIN_DATA)   ? ">" : " ", 2);
    Lcd_Text(4, 100, (g_cursor == MAIN_FILTER) ? ">" : " ", 2);
    Lcd_Text(4, 140, (g_cursor == MAIN_ATT)    ? ">" : " ", 2);
    /* 采样计数行（10Hz刷新，采样数持续自增即证明后台100Hz采集未停） */
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    strcpy(line, "S:");
    ufmt(num, g_snap.sample_count, 0); strcat(line, num);
    Lcd_Text(4, 184, line, 30);
    /* 姿态角行 */
    NEUQ_LCD200_SetPenColor(LCD_CYAN);
    strcpy(line, "R:");
    ffmt(b, g_snap.roll_deg, 6, 2);  strcat(line, b);
    strcat(line, " P:");
    ffmt(b, g_snap.pitch_deg, 6, 2); strcat(line, b);
    strcat(line, " Y:");
    ffmt(b, g_snap.yaw_deg, 6, 2);   strcat(line, b);
    Lcd_Text(4, 204, line, 30);
}
//---------------- 六轴数据页 ----------------
static void Page_Data_Frame(void)
{
    NEUQ_LCD200_SetTextFont(&ASCII_Font20);
    NEUQ_LCD200_SetPenColor(LCD_CYAN);
    Lcd_Text(4, 6, "Six-axis data", 14);
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_YELLOW);
    Lcd_Text(4,  30, "Axis", 7);
    Lcd_Text(60, 30, "Raw", 7);
    Lcd_Text(116,30, "Unit", 5);
    Lcd_Text(152,30, "Filt", 7);
    Lcd_Text(208,30, "Unit", 4);
    /* 六行轴名为静态内容，进页绘制一次 */
    Lcd_Text(4, 44,  "AccX", 7);
    Lcd_Text(4, 66,  "AccY", 7);
    Lcd_Text(4, 88,  "AccZ", 7);
    Lcd_Text(4, 110, "GyrX", 7);
    Lcd_Text(4, 132, "GyrY", 7);
    Lcd_Text(4, 154, "GyrZ", 7);
    NEUQ_LCD200_SetPenColor(LCD_GREY);
    Lcd_Text(4, 236, "Up/Dn axis  OK send  MENU back", 30);
}
static void Page_Data_Refresh(void)
{
    uint8_t i;
    uint16_t y;
    float raw_v, flt_v;
    float fc;
    uint8_t dec;
    const char *unit;
    char rb[8], fb[8], line[40], num[12], tb[8], ab[8], fcb[8];
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    /* 六行数值：原始值与滤波值同帧全量刷新（轴名进页时已静态绘制） */
    for (i = 0; i < 6u; i++)
    {
        y = 44u + (uint16_t)i * 22u;
        raw_v = (i < 3u) ? g_snap.acc_g[i] : g_snap.gyro_dps[i - 3u];
        flt_v = (i < 3u) ? g_snap.acc_g_flt[i] : g_snap.gyro_dps_flt[i - 3u];
        dec = (i < 3u) ? 3u : 2u;
        unit = (i < 3u) ? "g" : "d/s";
        NEUQ_LCD200_SetPenColor(LCD_WHITE);
        ffmt(rb, raw_v, 7, dec);
        Lcd_Text(60, y, rb, 7);
        Lcd_Text(116, y, unit, 5);
        NEUQ_LCD200_SetPenColor(LCD_GREEN);
        ffmt(fb, flt_v, 7, dec);
        Lcd_Text(152, y, fb, 7);
        Lcd_Text(208, y, unit, 4);
    }
    /* 滤波系数/截止频率行 */
    fc = 0.15915f * g_snap.filter_alpha / 0.01f;
    NEUQ_LCD200_SetPenColor(LCD_CYAN);
    strcpy(line, "alpha=");
    ffmt(ab, g_snap.filter_alpha, 0, 3); strcat(line, ab);
    strcat(line, " fc=");
    ffmt(fcb, fc, 0, 1); strcat(line, fcb);
    strcat(line, "Hz");
    Lcd_Text(4, 176, line, 28);
    /* 采样数/温度行 */
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    strcpy(line, "S:");
    ufmt(num, g_snap.sample_count, 0); strcat(line, num);
    strcat(line, " T:");
    ffmt(tb, g_snap.temperature, 0, 1); strcat(line, tb);
    strcat(line, "C");
    Lcd_Text(4, 196, line, 28);
    /* VOFA状态行 */
    NEUQ_LCD200_SetPenColor(g_snap.upload_enable ? LCD_GREEN : LCD_GREY);
    if (g_snap.upload_enable)
    {
        strcpy(line, "VOFA ON : ");
        strcat(line, Axis_Name(g_snap.upload_axis));
        strcat(line, " raw,filt");
    }
    else
    {
        strcpy(line, "VOFA OFF (OK to send)");
    }
    Lcd_Text(4, 216, line, 28);
}
//---------------- 波形与滤波设置页 ----------------
static void Page_Filter_Frame(void)
{
    NEUQ_LCD200_SetTextFont(&ASCII_Font20);
    NEUQ_LCD200_SetPenColor(LCD_CYAN);
    Lcd_Text(4, 6, "Wave & Filter", 15);
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    Lcd_Text(28, 60,  "1.Axis :", 10);
    Lcd_Text(28, 100, "2.Send :", 10);
    Lcd_Text(28, 140, "3.Alpha:", 10);
    Lcd_Text(28, 178, "4.Save :", 10);
    Lcd_Text(28, 218, "5.Back", 10);
    Lcd_Text(4, 262, "Up/Dn move  OK action", 26);
    Lcd_Text(4, 282, "Axis/Alpha: OK edit Up/Dn", 26);
}
static void Page_Filter_Refresh(void)
{
    char line[32], ab[10], fcb[8];
    float fc;
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    /* 光标（编辑项显示 E） */
    NEUQ_LCD200_SetPenColor(LCD_GREEN);
    Lcd_Text(4, 60,  (g_flt_cursor == 0u) ? (g_edit == 1u ? "E" : ">") : " ", 2);
    Lcd_Text(4, 100, (g_flt_cursor == 1u) ? ">" : " ", 2);
    Lcd_Text(4, 140, (g_flt_cursor == 2u) ? (g_edit == 2u ? "E" : ">") : " ", 2);
    Lcd_Text(4, 178, (g_flt_cursor == 3u) ? ">" : " ", 2);
    Lcd_Text(4, 218, (g_flt_cursor == 4u) ? ">" : " ", 2);
    /* 上传轴（编辑态加 * 提示） */
    NEUQ_LCD200_SetPenColor(g_edit == 1u ? LCD_YELLOW : LCD_WHITE);
    strcpy(line, Axis_Name(g_snap.upload_axis));
    if (g_edit == 1u) strcat(line, " *");
    Lcd_Text(112, 60, line, 16);
    /* 发送开关状态 */
    NEUQ_LCD200_SetPenColor(g_snap.upload_enable ? LCD_GREEN : LCD_GREY);
    Lcd_Text(112, 100, g_snap.upload_enable ? "ON " : "OFF", 16);
    /* 滤波系数（编辑态加 * 提示） */
    fc = 0.15915f * g_snap.filter_alpha / 0.01f;
    NEUQ_LCD200_SetPenColor(g_edit == 2u ? LCD_YELLOW : LCD_WHITE);
    ffmt(ab, g_snap.filter_alpha, 0, 3);
    strcpy(line, ab);
    if (g_edit == 2u) strcat(line, "*");
    strcat(line, " ");
    ffmt(fcb, fc, 0, 1);
    strcat(line, fcb);
    strcat(line, "Hz");
    Lcd_Text(112, 140, line, 16);
    /* Flash保存状态 */
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    switch (g_flash_st)
    {
        case FLASH_ST_BUSY: NEUQ_LCD200_SetPenColor(LCD_YELLOW); strcpy(line, "Saving..."); break;
        case FLASH_ST_OK:   NEUQ_LCD200_SetPenColor(LCD_GREEN);  strcpy(line, "Saved OK");  break;
        case FLASH_ST_FAIL: NEUQ_LCD200_SetPenColor(LCD_RED);    strcpy(line, "Save FAIL"); break;
        default:            strcpy(line, g_flash_ok ? "Press OK" : "No Flash"); break;
    }
    Lcd_Text(112, 178, line, 12);
    /* 底部操作提示随编辑态切换 */
    NEUQ_LCD200_SetPenColor(g_edit ? LCD_YELLOW : LCD_GREY);
    if (g_edit == 1u)
    {
        Lcd_Text(4, 302, "Up/Dn change axis  OK end", 30);
    }
    else if (g_edit == 2u)
    {
        Lcd_Text(4, 302, "Up/Dn alpha +-0.005 OK end", 30);
    }
    else
    {
        Lcd_Text(4, 302, "Saved params restore at boot", 30);
    }
}
//---------------- 姿态解算页 ----------------
static void Page_Att_Frame(void)
{
    NEUQ_LCD200_SetTextFont(&ASCII_Font20);
    NEUQ_LCD200_SetPenColor(LCD_CYAN);
    Lcd_Text(4, 6, "Attitude AHRS", 15);
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_YELLOW);
    Lcd_Text(4,  96, "Roll", 5);
    Lcd_Text(84, 96, "Pitch", 6);
    Lcd_Text(168,96, "Yaw", 4);
    NEUQ_LCD200_SetPenColor(LCD_GREY);
    Lcd_Text(4, 156, "GyroBias(dps)", 14);
    Lcd_Text(4, 276, "MENU back", 30);
}
static void Page_Att_Refresh(void)
{
    char b[10], line[32];
    char xb[10], yb[10], zb[10];
    char q0[10], q1[10], q2[10], q3[10];
    /* 三个大角度（任务书要求动态显示横滚/俯仰/航向角） */
    NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    NEUQ_LCD200_SetPenColor(LCD_GREEN);
    ffmt(b, g_snap.roll_deg, 7, 2);
    Lcd_Text(4, 120, b, 7);
    ffmt(b, g_snap.pitch_deg, 7, 2);
    Lcd_Text(84, 120, b, 7);
    ffmt(b, g_snap.yaw_deg, 6, 2);
    Lcd_Text(168, 120, b, 6);
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    /* 陀螺零偏（标定结果，答辩时可直接说明零偏校准效果） */
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    strcpy(line, "X:");
    ffmt(xb, g_snap.gyro_bias[0], 0, 3); strcat(line, xb);
    strcat(line, " Y:");
    ffmt(yb, g_snap.gyro_bias[1], 0, 3); strcat(line, yb);
    strcat(line, " Z:");
    ffmt(zb, g_snap.gyro_bias[2], 0, 3); strcat(line, zb);
    Lcd_Text(4, 176, line, 30);
    /* 四元数两行（任务书要求基于四元数的姿态更新，直接可见证明） */
    NEUQ_LCD200_SetPenColor(LCD_MAGENTA);
    strcpy(line, "q0=");
    ffmt(q0, g_snap.quat[0], 0, 4); strcat(line, q0);
    strcat(line, " q1=");
    ffmt(q1, g_snap.quat[1], 0, 4); strcat(line, q1);
    Lcd_Text(4, 198, line, 30);
    strcpy(line, "q2=");
    ffmt(q2, g_snap.quat[2], 0, 4); strcat(line, q2);
    strcat(line, " q3=");
    ffmt(q3, g_snap.quat[3], 0, 4); strcat(line, q3);
    Lcd_Text(4, 216, line, 30);
    /* 姿态更新周期说明（100Hz，与任务书“能够说明姿态更新周期”对应） */
    NEUQ_LCD200_SetPenColor(LCD_GREEN);
    if (g_snap.mpu_ok)
    {
        Lcd_Text(4, 244, "Calib DONE  100Hz update", 30);
    }
    else
    {
        NEUQ_LCD200_SetPenColor(LCD_RED);
        Lcd_Text(4, 244, "MPU6050 NOT FOUND", 30);
    }
}
//---------------- 页面切换与统一刷新入口 ----------------
static void Page_Enter(Page_Id_e page)
{
    g_page = page;
    // 进页先绘静态Frame，动态区由后续刷新周期绘制
    switch (page)
    {
        case PAGE_BOOT:   Page_Boot_Draw();    break;
        case PAGE_MAIN:   Page_Main_Frame();   break;
        case PAGE_DATA:   Page_Data_Frame();   break;
        case PAGE_FILTER: Page_Filter_Frame(); break;
        case PAGE_ATT:    Page_Att_Frame();    break;
        default: break;
    }
    g_need_refresh = 1;
}
void Page_Refresh(Page_Id_e page)
{
    switch (page)
    {
        case PAGE_BOOT:   Page_Boot_Refresh();  break;
        case PAGE_MAIN:   Page_Main_Refresh();  break;
        case PAGE_DATA:   Page_Data_Refresh();  break;
        case PAGE_FILTER: Page_Filter_Refresh();break;
        case PAGE_ATT:    Page_Att_Refresh();   break;
        default: break;
    }
}
//===========================================================//
//===== 七、菜单按键逻辑（事件驱动；无长按连发） =====
//===========================================================//
static void Menu_Process(void)
{
    /* ---------- 主菜单 ---------- */
    if (g_page == PAGE_MAIN)
    {
        if (Key_Take(KEY_UP))   { if (g_cursor == 0) g_cursor = MAIN_ITEM_NUM - 1u; else g_cursor--; g_need_refresh = 1; }
        if (Key_Take(KEY_DOWN)) { g_cursor = (g_cursor + 1u) % MAIN_ITEM_NUM; g_need_refresh = 1; }
        if (Key_Take(KEY_OK))
        {
            if (g_cursor == MAIN_FILTER)
            {
                g_flt_cursor = 0;
                g_edit = 0;
            }
            Page_Enter((Page_Id_e)(PAGE_DATA + g_cursor));
            return;
        }
        if (Key_Take(KEY_BACK)) { g_cursor = MAIN_DATA; g_need_refresh = 1; }
    }
    /* ---------- 六轴数据页 ---------- */
    else if (g_page == PAGE_DATA)
    {
        if (Key_Take(KEY_UP))
        {
            uint8_t ax = (g_snap.upload_axis == 0u) ? 5u : (uint8_t)(g_snap.upload_axis - 1u);
            IMU_Set_UploadAxis(ax);
            g_need_refresh = 1;
        }
        if (Key_Take(KEY_DOWN))
        {
            IMU_Set_UploadAxis((uint8_t)((g_snap.upload_axis + 1u) % 6u));
            g_need_refresh = 1;
        }
        if (Key_Take(KEY_OK))
        {
            IMU_Set_Upload(!g_snap.upload_enable);   // 只切换发送，采集/滤波/解算不停止
            g_need_refresh = 1;
        }
        if (Key_Take(KEY_BACK)) { Page_Enter(PAGE_MAIN); return; }
    }
    /* ---------- 波形与滤波设置页：五项光标，参数编辑态 ---------- */
    else if (g_page == PAGE_FILTER)
    {
        if (g_edit != 0u)
        {
            /* 编辑态：UP/DN 修改被编辑量，OK/BACK 结束编辑（不直接退页） */
            if (Key_Take(KEY_UP))
            {
                if (g_edit == 1u)
                {
                    IMU_Set_UploadAxis((g_snap.upload_axis == 0u) ? 5u
                                                                 : (uint8_t)(g_snap.upload_axis - 1u));
                }
                else
                {
                    IMU_Set_Alpha(g_snap.filter_alpha + ALPHA_STEP);
                }
                g_need_refresh = 1;
            }
            if (Key_Take(KEY_DOWN))
            {
                if (g_edit == 1u)
                {
                    IMU_Set_UploadAxis((uint8_t)((g_snap.upload_axis + 1u) % 6u));
                }
                else
                {
                    IMU_Set_Alpha(g_snap.filter_alpha - ALPHA_STEP);
                }
                g_need_refresh = 1;
            }
            uint8_t ok_event   = Key_Take(KEY_OK);
            uint8_t back_event = Key_Take(KEY_BACK);
            if (ok_event || back_event)
            {
                g_edit = 0;
                g_need_refresh = 1;
            }
        }
        else
        {
            /* 非编辑态：UP/DN 移动光标，OK 执行各项动作 */
            if (Key_Take(KEY_UP))
            {
                g_flt_cursor = (g_flt_cursor == 0u) ? 4u : (uint8_t)(g_flt_cursor - 1u);
                g_need_refresh = 1;
            }
            if (Key_Take(KEY_DOWN))
            {
                g_flt_cursor = (uint8_t)((g_flt_cursor + 1u) % 5u);
                g_need_refresh = 1;
            }
            if (Key_Take(KEY_OK))
            {
                switch (g_flt_cursor)
                {
                    case 0: g_edit = 1; break;                    // 进入轴编辑
                    case 1: IMU_Set_Upload(!g_snap.upload_enable);
                            break;                               // 发送开关
                    case 2: g_edit = 2; break;                    // 进入alpha编辑
                    case 3: if (g_flash_st != FLASH_ST_BUSY) Param_SaveNow();
                            break;                               // 保存到Flash
                    case 4: Page_Enter(PAGE_MAIN); return;       // 返回
                    default: break;
                }
                g_need_refresh = 1;
            }
            if (Key_Take(KEY_BACK)) { Page_Enter(PAGE_MAIN); return; }
        }
    }
    /* ---------- 姿态解算页 ---------- */
    else if (g_page == PAGE_ATT)
    {
        if (Key_Take(KEY_BACK)) { Page_Enter(PAGE_MAIN); return; }
    }
}
//===========================================================//
//===== 八、用户主函数 =====
//===========================================================//
void User_Main(void)
{
    uint8_t mpu_ok;
    uint32_t now;
    uint8_t proc;
    char ab[12];
    // **************************** 初始化 ****************************
    NEUQ_Debug_init();
    NEUQ_Delay_init();
    Key_Init();
    NEUQ_FIFO_init(&g_rx_fifo, g_rx_fifo_mem, sizeof(g_rx_fifo_mem));
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetDirection(LCD200_DIR_Parallel);
    NEUQ_LCD200_SetBackColor(LCD_BLACK);
    NEUQ_LCD200_Clear();
    g_flash_ok = (NEUQ_Flash_init() == NEUQ_FLASH_OK) ? 1u : 0u;
    printf("Flash init: %s\r\n", g_flash_ok ? "OK" : "FAIL");
    mpu_ok = NEUQ_MPU6050_init();
    printf("MPU6050 init: %s (I2C3 PA8/PC9, AD0=GND 0x68)\r\n", mpu_ok ? "OK" : "FAIL");
    /* 数据管线：环形缓冲/滤波/标定/解算状态初始化（必须在PIT启动前完成） */
    IMU_Pipeline_Init(mpu_ok);
    Param_Load();
    IMU_GetSnapshot(&g_snap);
    ffmt(ab, g_snap.filter_alpha, 0, 3);
    printf("Filter alpha=%s axis=%u, VOFA off at boot.\r\n", ab, (unsigned)g_snap.upload_axis);
    /* SysTick保持最高优先级：PIT内I2C阻塞期间 HAL_GetTick 仍可走，
     * HAL_I2C超时才能正常返回，避免超时死等把系统时基拖死 */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    /* 开机画面：先显示“Calibrating...”，随后执行阻塞标定（PIT尚未启动，I2C无并发冲突）；
     * 任务书要求“完成静置校准”，开机静置2s即为操作说明 */
    g_boot_active = 1;
    g_boot_ref_tick = HAL_GetTick();
    Page_Enter(PAGE_BOOT);
    Page_Boot_Refresh();
    IMU_Calib_Blocking();               // 阻塞2s：200点零偏采集与平均
    g_calib_done = mpu_ok ? 1u : 0u;
    Page_Boot_Refresh();
    g_boot_tick = HAL_GetTick();        // 标定完成后开机页再停留800ms
    /* PIT_CH1 10ms采样中断；库默认优先级0，降到5避免与I2C/HAL内部时序相互阻塞 */
    NEUQ_PIT_init_ms(PIT_CH1, 10);
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    printf("PIT 10ms sampling started. ISR: read+FIFO only.\r\n");
    // **************************** 主循环 ****************************
    while (1)
    {
        now = HAL_GetTick();
        /* 1) 数据管线：每轮最多处理4帧；处理一帧即按该帧数据发送一对VOFA波形，
         *    保证“原始值/滤波值”严格来自同一次采样、同一时基、同一量纲 */
        proc = 0;
        while (proc < PIPELINE_MAX_LOOP)
        {
            if (!IMU_Pipeline_RunOnce()) break;
            IMU_GetSnapshot(&g_snap);
            VOFA_SendPair(&g_snap);
            proc++;
        }
        if (proc == 0u)
        {
            IMU_GetSnapshot(&g_snap);
        }
        /* 2) 按键状态机扫描（5ms周期，与页面/标定互不阻塞） */
        if (now - g_key_tick >= KEY_SCAN_MS)
        {
            g_key_tick = now;
            Key_Scan();
        }
        /* 4) 开机画面非阻塞停留（标定已在进入主循环前阻塞完成） */
        if (g_boot_active)
        {
            if (now - g_boot_tick >= BOOT_HOLD_MS)
            {
                g_boot_active = 0;
                Key_FlushAll();            // 开机期间的按键不作为菜单输入
                Page_Enter(PAGE_MAIN);
            }
            else if (now - g_boot_ref_tick >= LCD_REFRESH_MS)
            {
                g_boot_ref_tick = now;
                Page_Boot_Refresh();
            }
        }
        else
        {
            /* 5) Flash结果提示2s自动还原 */
            if (g_flash_st != FLASH_ST_IDLE && (now - g_flash_tick) >= FLASH_RESULT_MS)
            {
                g_flash_st = FLASH_ST_IDLE;
                g_need_refresh = 1;
            }
            /* 6) 菜单事件处理 */
            Menu_Process();
            /* 7) LCD刷新：事件触发立即刷，否则100ms周期全量刷新（库显示自带背景填充，无闪烁） */
            if (g_need_refresh || (now - g_ref_tick >= LCD_REFRESH_MS))
            {
                g_ref_tick = now;
                g_need_refresh = 0;
                Page_Refresh(g_page);
            }
        }
    }
}
