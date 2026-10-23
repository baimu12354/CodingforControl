/*
 * NEUQ_MENU.c
 * 二级菜单界面：一级功能选择页 + 六轴数据 / 波形滤波设置 / 姿态解算三个二级页面
 *
 * 几条写界面的规矩：
 *   1) 菜单自己不留"影子状态"。选轴、发送开关、滤波参数全部直接读写g_setting，
 *      屏幕上显示的就是后台真正在用的值，不会出现显示和实际对不上的情况。
 *   2) 静态文字（标题、轴名、单位）只在进页面时画一次，周期刷新只重画会变的数值。
 *      LCD200走SPI，整屏重画要几十毫秒，只画数值域界面才跟手，也不拖慢采样。
 *   3) 数值都按固定宽度格式化（比如%+7.3f），长度不变就不用先擦再写，省一半像素传输。
 *
 * 屏幕横屏320x240，标题用ASCII_Font24（12x24），正文用ASCII_Font16（8x16），
 * 正文一行40个字符，一共15行。
 */
#include "NEUQ_lib_include.h"
#include "NEUQ_Setting.h"
#include "NEUQ_IMU.h"
#include "NEUQ_Attitude.h"
#include "NEUQ_Wave.h"
#include "NEUQ_MENU.h"

#define MENU_SCR_W              (320)
#define MENU_SCR_H              (240)

// 按键事件调试打印，1打开0关闭，排错用
// 格式 [KEY] ev=事件号 page=页面 cur=光标 edit=是否在编辑
// 事件号 1=UP短 2=UP长 3=DOWN短 4=DOWN长 5=OK短 6=OK长 7=BACK短 8=BACK长
// 这里不能写死1：波形上传要是也在UART_0，两者会抢同一个串口，
// 打印的文字混进波形数据里，上位机收到的就是乱码
#if (NEUQ_WAVE_UART == UART_0)
#define NEUQ_MENU_KEY_DEBUG     (0)     // 波形占了UART_0，调试打印让路
#else
#define NEUQ_MENU_KEY_DEBUG     (1)     // 波形在别的串口上，UART_0空着，可以打
#endif

// 往调试串口USART1(PA9)打一行字，自带超时，不会卡住主循环
// 不用printf是因为库里的NEUQ_Uart_write_byte()是 while(!(USARTx->SR & USART_SR_TXE));
// 没有超时，printf底层就是调它。这段打印在主循环里每按一次键就跑一次，
// 万一USART1不再置TXE，按一下键就把整个单片机卡死了，拔线也不恢复，只能复位。
// 所以这里自己写，等不到就放弃，最坏少打一行字。
void NEUQ_Menu_DebugPrint(const char *text)
{
    uint32_t i;
    uint32_t t0;

    if(text == 0) return;

    for(i = 0; text[i] != '\0'; i++)
    {
        t0 = DWT->CYCCNT;
        while((USART1->SR & USART_SR_TXE) == 0)
        {
            if((uint32_t)(DWT->CYCCNT - t0) > (SystemCoreClock / 1000U))    // 1ms超时
            {
                return;                                                     // 放弃
            }
        }
        USART1->DR = (uint8_t)text[i];
    }
}

// ---- 一级页面的坐标 ----
#define MP_Y_TITLE              (6)
#define MP_Y_ITEM0              (68)
#define MP_ITEM_H               (36)
#define MP_Y_HINT               (190)
#define MP_Y_INFO               (212)
#define MP_X_LEFT               (8)

// ---- 六轴数据显示页的坐标 ----
#define DP_Y_TITLE              (2)
#define DP_Y_HEADER             (56)
#define DP_Y_ROW0               (96)
#define DP_ROW_H                (40)
#define DP_X_LABEL_A            (24)
#define DP_X_VAL_A              (56)        // 7字符宽 = 56像素
#define DP_X_LABEL_G            (168)
#define DP_X_VAL_G              (200)
#define DP_Y_FOOTER             (210)

// ---- 波形与滤波设置页的坐标 ----
#define WP_Y_TITLE              (2)
#define WP_Y_ROW0               (30)
#define WP_ROW_H                (20)
#define WP_Y_FOOTER             (212)
#define WP_X_STATUS             (200)       // 状态文字起点，左边留给操作提示，最多24字符

// ---- 姿态解算页的坐标 ----
#define AP_Y_TITLE              (2)
#define AP_Y_ROW0               (32)        // 共6行：三个角 + 空一行 + 两行四元数
#define AP_ROW_H                (22)
#define AP_X_VAL                (64)        // 姿态角数值域，7字符宽
#define AP_X_UNIT               (120)
#define AP_X_Q0                 (24)        // 四元数q0/q2数值域
#define AP_X_Q1                 (120)       // 四元数q1/q3数值域
#define AP_Y_NOTE               (176)
#define AP_Y_FOOTER             (206)

static MENU_PAGE_ENUM s_page       = MENU_PAGE_MAIN;
static uint8_t        s_main_cur   = 0;     // 一级页面光标0~2，切页后保持
static uint8_t        s_wave_cur   = 0;     // 设置页光标0~8，切页后保持
static uint8_t        s_editing    = 0;     // 1表示设置页正在编辑参数

static uint8_t        s_bk_u8      = 0;     // 编辑前的整数参数副本
static float          s_bk_f       = 0.0f;  // 编辑前的浮点参数副本

static char           s_status[24] = "READY";           // 设置页右下角状态
static char           s_status_shown[24] = "";          // 已经画到屏幕上的那个

static uint8_t        s_send_shown = 0;     // 上次刷屏时Send行显示的是不是ON

static const char *kMainItems[3] = {"1. AXIS DATA", "2. WAVE & FILTER", "3. ATTITUDE"};

enum{
    W_ITEM_AXIS = 0,
    W_ITEM_SEND,
    W_ITEM_RATE,
    W_ITEM_ACC_FC,
    W_ITEM_GYRO_FC,
    W_ITEM_SAVE,
    W_ITEM_RESET,
    W_ITEM_NUM
};
static const char *kWaveLabels[W_ITEM_NUM] = {
    "Axis", "Send", "Rate", "ACC fc", "GYRO fc", "Save", "Reset"
};

static void Lcd_TitleFont(void)
{
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font24);
}

static void Lcd_BodyFont(void)
{
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
}

// 画字符串，画之前先自己选好字体和颜色
static void Lcd_Str(uint16_t x, uint16_t y, char *s)
{
    if(x >= MENU_SCR_W || y >= MENU_SCR_H) return;
    NEUQ_LCD200_DisplayString(x, y, s);
}

// 参数一改就立刻通知各模块，做到改完马上能看到效果
static void Menu_ApplySetting(void)
{
    NEUQ_IMU_SetFilterHz(g_setting.acc_lpf_hz, g_setting.gyro_lpf_hz);
    NEUQ_Attitude_SetGains(g_setting.att_kp, g_setting.att_ki);
    NEUQ_Wave_ResetPhase();
}

/*==================== 一级页面：功能选择 ====================*/
static void Main_DrawItem(uint8_t idx)
{
    char buf[26];
    Lcd_TitleFont();
    snprintf(buf, sizeof(buf), "%c %s", (idx == s_main_cur) ? '>' : ' ', kMainItems[idx]);
    Lcd_Str(MP_X_LEFT, (uint16_t)(MP_Y_ITEM0 + idx * MP_ITEM_H), buf);
}

static void Main_Draw(void)
{
    NEUQ_LCD200_Clear();

    Lcd_TitleFont();
    Lcd_Str(MP_X_LEFT, MP_Y_TITLE, "GYRO 6-AXIS SYSTEM");

    Lcd_BodyFont();
    Lcd_Str(MP_X_LEFT, 36, "MPU6050  I2C3  500Hz  LPF");
    Lcd_Str(MP_X_LEFT, MP_Y_HINT, "UP/DN: MOVE   OK: ENTER");

    if(g_setting_valid)
    {
        Lcd_Str(MP_X_LEFT, MP_Y_INFO, "PARAM: FROM FLASH");
    }
    else
    {
        Lcd_Str(MP_X_LEFT, MP_Y_INFO, "PARAM: DEFAULT   ");
    }

    Main_DrawItem(0);
    Main_DrawItem(1);
    Main_DrawItem(2);

    s_status_shown[0] = '\0';       // 换页后状态区要重画
}

static void Main_HandleKey(KEY_EVENT_ENUM ev)
{
    switch(ev)
    {
        case KEY_UP_SHORT:
        case KEY_UP_LONG:
            s_main_cur = (uint8_t)((s_main_cur + 2) % 3);
            Main_DrawItem(s_main_cur);
            Main_DrawItem((uint8_t)((s_main_cur + 1) % 3));
            break;

        case KEY_DOWN_SHORT:
        case KEY_DOWN_LONG:
            s_main_cur = (uint8_t)((s_main_cur + 1) % 3);
            Main_DrawItem(s_main_cur);
            Main_DrawItem((uint8_t)((s_main_cur + 2) % 3));
            break;

        case KEY_FORWARD_SHORT:
        case KEY_FORWARD_LONG:
            switch(s_main_cur)
            {
                case 0: s_page = MENU_PAGE_DATA; break;
                case 1: s_page = MENU_PAGE_WAVE; break;
                default: s_page = MENU_PAGE_ATT; break;
            }
            NEUQ_Menu_Init();       // 重画目标页面
            break;

        default:
            break;
    }
}

/*==================== 六轴数据显示页 ====================*/
// 左边加速度XYZ，右边角速度XYZ，六个滤波后的物理量，别的都不显示
static void Data_DrawStatic(void)
{
    NEUQ_LCD200_Clear();

    Lcd_TitleFont();
    Lcd_Str(DP_X_LABEL_A, DP_Y_TITLE, "6-AXIS DATA");

    Lcd_BodyFont();
    Lcd_Str(DP_X_LABEL_A, DP_Y_HEADER, "ACCEL [g]");
    Lcd_Str(DP_X_LABEL_G, DP_Y_HEADER, "GYRO [dps]");

    // 轴名只画一次
    Lcd_Str(DP_X_LABEL_A, (uint16_t)(DP_Y_ROW0 + 0 * DP_ROW_H), "AX");
    Lcd_Str(DP_X_LABEL_A, (uint16_t)(DP_Y_ROW0 + 1 * DP_ROW_H), "AY");
    Lcd_Str(DP_X_LABEL_A, (uint16_t)(DP_Y_ROW0 + 2 * DP_ROW_H), "AZ");
    Lcd_Str(DP_X_LABEL_G, (uint16_t)(DP_Y_ROW0 + 0 * DP_ROW_H), "GX");
    Lcd_Str(DP_X_LABEL_G, (uint16_t)(DP_Y_ROW0 + 1 * DP_ROW_H), "GY");
    Lcd_Str(DP_X_LABEL_G, (uint16_t)(DP_Y_ROW0 + 2 * DP_ROW_H), "GZ");

    Lcd_Str(DP_X_LABEL_A, DP_Y_FOOTER, "BACK: RETURN");
}

static void Data_Update(void)
{
    char buf[16];
    uint8_t i;

    Lcd_BodyFont();

    for(i = 0; i < 3; i++)
    {
        uint16_t y = (uint16_t)(DP_Y_ROW0 + i * DP_ROW_H);

        // 固定7字符宽，刷新时直接盖住上一次的数字，不用先擦
        snprintf(buf, sizeof(buf), "%+7.3f", g_imu.acc_g[i]);
        Lcd_Str(DP_X_VAL_A, y, buf);

        snprintf(buf, sizeof(buf), "%+7.1f", g_imu.gyro_dps[i]);
        Lcd_Str(DP_X_VAL_G, y, buf);
    }
}

static void Data_HandleKey(KEY_EVENT_ENUM ev)
{
    if(ev == KEY_BACK_SHORT || ev == KEY_BACK_LONG)
    {
        s_page = MENU_PAGE_MAIN;
        NEUQ_Menu_Init();
    }
}

/*==================== 波形与滤波设置页 ====================*/
// 把某一条的当前值格式化成字符串
static void Wave_ValueText(uint8_t idx, char *buf, uint16_t len)
{
    switch(idx)
    {
        case W_ITEM_AXIS:
            snprintf(buf, len, "%s (%s)", NEUQ_Setting_AxisName(g_setting.wave_axis),
                                          NEUQ_Setting_AxisText(g_setting.wave_axis));
            break;
        case W_ITEM_SEND:
            // 发送状态后面跟一个已发送帧数的实时计数，四种情况都补到同样宽度，
            // 刷新时就不用先擦除，长度不变不会留残影。
            // 排查收不到波形的时候这就是第一手证据：
            //   计数在涨 => 单片机确实发出去了，问题在接线/波特率/上位机设置；
            //   计数不动 => 单片机这边就没发。
            // UART ERR表示等TXE超时，上传已被自动关掉，免得主循环卡死。
            if(g_setting.wave_enable)
            {
                snprintf(buf, len, "ON   #%6lu", (unsigned long)NEUQ_Wave_GetFrames());
            }
            else if(NEUQ_Wave_IsUartOk() == 0)
            {
                snprintf(buf, len, "OFF  UART ERR");
            }
            else
            {
                snprintf(buf, len, "OFF         ");
            }
            break;
        case W_ITEM_RATE:
            snprintf(buf, len, "%d Hz", (int)NEUQ_Setting_WaveRateHz(g_setting.wave_rate));
            break;
        case W_ITEM_ACC_FC:
            snprintf(buf, len, "%.0f Hz", g_setting.acc_lpf_hz);
            break;
        case W_ITEM_GYRO_FC:
            snprintf(buf, len, "%.0f Hz", g_setting.gyro_lpf_hz);
            break;
        case W_ITEM_SAVE:
            snprintf(buf, len, "press OK");
            break;
        default:
            snprintf(buf, len, "press OK");
            break;
    }
}

static void Wave_DrawRow(uint8_t idx)
{
    char buf[48];
    char val[24];
    char mark;

    if(idx >= W_ITEM_NUM) return;

    if(idx == s_wave_cur)
    {
        mark = s_editing ? '*' : '>';
    }
    else
    {
        mark = ' ';
    }

    Wave_ValueText(idx, val, sizeof(val));
    snprintf(buf, sizeof(buf), "%c %-7s: %s", mark, kWaveLabels[idx], val);

    Lcd_BodyFont();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    Lcd_Str(0, (uint16_t)(WP_Y_ROW0 + idx * WP_ROW_H), buf);
}

// 移动光标只需要重画旧光标行和新光标行，不用整屏重画，翻菜单才跟手
static void Wave_DrawTwo(uint8_t a, uint8_t b)
{
    Wave_DrawRow(a);
    if(b != a)
    {
        Wave_DrawRow(b);
    }
}

static void Wave_DrawFooter(void);      // 定义在下面

static void Wave_DrawStatic(void)
{
    uint8_t i;

    NEUQ_LCD200_Clear();

    Lcd_TitleFont();
    Lcd_Str(MP_X_LEFT, WP_Y_TITLE, "WAVE & FILTER");

    Lcd_BodyFont();

    for(i = 0; i < W_ITEM_NUM; i++)
    {
        Wave_DrawRow(i);
    }
    Wave_DrawFooter();
    s_status_shown[0] = '\0';       // 强制重画一次状态
}

static void Wave_DrawStatus(void)
{
    Lcd_BodyFont();
    if(g_setting_dirty)
    {
        NEUQ_LCD200_SetPenColor(LCD_RED);
    }
    else
    {
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
    }
    Lcd_Str(WP_X_STATUS, WP_Y_FOOTER, s_status);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    snprintf(s_status_shown, sizeof(s_status_shown), "%s", s_status);
}

// 底部操作提示，让当前处在哪个模式一眼就能看出来
//   浏览：OK:EDIT  BACK:RET
//   编辑：EDIT: UP/DN ADJ  OK:SET   这时UP/DN才是加减参数，蓝字，行首还有*
// 两行都是23个字符，切来切去不会留残影
static void Wave_DrawFooter(void)
{
    Lcd_BodyFont();

    if(s_editing)
    {
        NEUQ_LCD200_SetPenColor(LCD_BLUE);
        Lcd_Str(0, WP_Y_FOOTER, "EDIT: UP/DN ADJ  OK:SET");
    }
    else
    {
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        Lcd_Str(0, WP_Y_FOOTER, "OK:EDIT  BACK:RET      ");
    }
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
}

// 开始编辑前把这一条的旧值存一份
static void Wave_BackupItem(void)
{
    switch(s_wave_cur)
    {
        case W_ITEM_AXIS:    s_bk_u8 = g_setting.wave_axis;   break;
        case W_ITEM_SEND:    s_bk_u8 = g_setting.wave_enable; break;
        case W_ITEM_RATE:    s_bk_u8 = g_setting.wave_rate;   break;
        case W_ITEM_ACC_FC:  s_bk_f  = g_setting.acc_lpf_hz;  break;
        case W_ITEM_GYRO_FC: s_bk_f  = g_setting.gyro_lpf_hz; break;
        default: break;
    }
}

// BACK取消编辑，把旧值放回去
static void Wave_RestoreItem(void)
{
    switch(s_wave_cur)
    {
        case W_ITEM_AXIS:    g_setting.wave_axis   = s_bk_u8; break;
        case W_ITEM_SEND:    g_setting.wave_enable = s_bk_u8; break;
        case W_ITEM_RATE:    g_setting.wave_rate   = s_bk_u8; break;
        case W_ITEM_ACC_FC:  g_setting.acc_lpf_hz  = s_bk_f;  break;
        case W_ITEM_GYRO_FC: g_setting.gyro_lpf_hz = s_bk_f;  break;
        default: break;
    }
    Menu_ApplySetting();
}

static void Wave_Adjust(int8_t dir)
{
    switch(s_wave_cur)
    {
        case W_ITEM_AXIS:
            g_setting.wave_axis = (uint8_t)((g_setting.wave_axis + NEUQ_AXIS_NUM + dir) % NEUQ_AXIS_NUM);
            break;

        case W_ITEM_SEND:
            g_setting.wave_enable = (g_setting.wave_enable == 0) ? 1 : 0;
            if(g_setting.wave_enable) NEUQ_Wave_ResetPhase();
            break;

        case W_ITEM_RATE:
            g_setting.wave_rate = (uint8_t)((g_setting.wave_rate + NEUQ_WAVE_RATE_NUM + dir) % NEUQ_WAVE_RATE_NUM);
            NEUQ_Wave_ResetPhase();
            break;

        case W_ITEM_ACC_FC:
            g_setting.acc_lpf_hz += (float)dir;
            if(g_setting.acc_lpf_hz < NEUQ_SETTING_FC_MIN) g_setting.acc_lpf_hz = NEUQ_SETTING_FC_MIN;
            if(g_setting.acc_lpf_hz > NEUQ_SETTING_FC_MAX) g_setting.acc_lpf_hz = NEUQ_SETTING_FC_MAX;
            NEUQ_IMU_SetFilterHz(g_setting.acc_lpf_hz, g_setting.gyro_lpf_hz);
            break;

        case W_ITEM_GYRO_FC:
            g_setting.gyro_lpf_hz += (float)dir;
            if(g_setting.gyro_lpf_hz < NEUQ_SETTING_FC_MIN) g_setting.gyro_lpf_hz = NEUQ_SETTING_FC_MIN;
            if(g_setting.gyro_lpf_hz > NEUQ_SETTING_FC_MAX) g_setting.gyro_lpf_hz = NEUQ_SETTING_FC_MAX;
            NEUQ_IMU_SetFilterHz(g_setting.acc_lpf_hz, g_setting.gyro_lpf_hz);
            break;

        default:
            break;
    }
}

// 执行存Flash和恢复默认这两个动作
static void Wave_DoAction(uint8_t idx)
{
    if(idx == W_ITEM_SAVE)
    {
        NEUQ_Menu_SetStatus("SAVING...");
        Wave_DrawStatus();

        if(NEUQ_Setting_Save() == NEUQ_FLASH_OK)
        {
            NEUQ_Menu_SetStatus("SAVED");
        }
        else
        {
            NEUQ_Menu_SetStatus("SAVE FAIL");
        }
        Wave_DrawStatus();
    }
    else if(idx == W_ITEM_RESET)
    {
        NEUQ_Setting_Default();
        Menu_ApplySetting();
        // NEUQ_Setting_Default()会把dirty清0，但这时RAM里的参数和Flash里的并不一样，
        // 所以这里重新置脏，让状态栏显示DEFAULT*，提示还没存盘
        g_setting_dirty = 1;
        Wave_DrawStatic();
        NEUQ_Menu_SetStatus("DEFAULT*");
        Wave_DrawStatus();
    }
}

static void Wave_HandleKey(KEY_EVENT_ENUM ev)
{
    switch(ev)
    {
        // ---- 光标移动 ----
        case KEY_UP_SHORT:
        case KEY_UP_LONG:
            if(s_editing)
            {
                Wave_Adjust(+1);
                Wave_DrawRow(s_wave_cur);
            }
            else
            {
                uint8_t prev = s_wave_cur;
                s_wave_cur = (uint8_t)((s_wave_cur + W_ITEM_NUM - 1) % W_ITEM_NUM);
                Wave_DrawTwo(prev, s_wave_cur);
            }
            break;

        case KEY_DOWN_SHORT:
        case KEY_DOWN_LONG:
            if(s_editing)
            {
                Wave_Adjust(-1);
                Wave_DrawRow(s_wave_cur);
            }
            else
            {
                uint8_t prev = s_wave_cur;
                s_wave_cur = (uint8_t)((s_wave_cur + 1) % W_ITEM_NUM);
                Wave_DrawTwo(prev, s_wave_cur);
            }
            break;

        // ---- 进入编辑 / 确认 / 执行动作 ----
        case KEY_FORWARD_SHORT:
        case KEY_FORWARD_LONG:
            if(s_editing)
            {
                // 确认修改，置脏标记，等Save那一条去落盘
                s_editing = 0;
                g_setting_dirty = 1;
                NEUQ_Menu_SetStatus("UNSAVED");
                Wave_DrawRow(s_wave_cur);
                Wave_DrawFooter();
                Wave_DrawStatus();
            }
            else if(ev == KEY_FORWARD_SHORT)
            {
                // 只有短按OK才进编辑或执行动作。
                // 长按OK走到这里什么都不做，这样以后就算重新打开OK的长按，
                // 也不会出现按久了一点就变成另一个功能的情况。
                if(s_wave_cur == W_ITEM_SAVE || s_wave_cur == W_ITEM_RESET)
                {
                    Wave_DoAction(s_wave_cur);
                }
                else
                {
                    // 开始编辑，先备份旧值，BACK的时候可以放弃修改
                    Wave_BackupItem();
                    s_editing = 1;
                    Wave_DrawRow(s_wave_cur);
                    Wave_DrawFooter();
                }
            }
            break;

        // ---- 返回上一级 / 放弃本次修改 ----
        case KEY_BACK_SHORT:
        case KEY_BACK_LONG:
            if(s_editing)
            {
                Wave_RestoreItem();
                s_editing = 0;
                Wave_DrawRow(s_wave_cur);
                Wave_DrawFooter();
            }
            else
            {
                s_page = MENU_PAGE_MAIN;
                NEUQ_Menu_Init();
            }
            break;

        default:
            break;
    }
}

/*==================== 姿态解算页 ====================*/
// 只留题目要的东西：三个姿态角、四元数，再加一行说明单位和更新周期。
// 零偏、Kp、dt-us、yaw_raw 这些是调参时看的中间量，不放屏幕上。
static void Att_DrawStatic(void)
{
    NEUQ_LCD200_Clear();

    Lcd_TitleFont();
    Lcd_Str(MP_X_LEFT, AP_Y_TITLE, "ATTITUDE");

    Lcd_BodyFont();
    Lcd_Str(0, (uint16_t)(AP_Y_ROW0 + 0 * AP_ROW_H), "Roll  :");
    Lcd_Str(AP_X_UNIT, (uint16_t)(AP_Y_ROW0 + 0 * AP_ROW_H), " deg");
    Lcd_Str(0, (uint16_t)(AP_Y_ROW0 + 1 * AP_ROW_H), "Pitch :");
    Lcd_Str(AP_X_UNIT, (uint16_t)(AP_Y_ROW0 + 1 * AP_ROW_H), " deg");
    Lcd_Str(0, (uint16_t)(AP_Y_ROW0 + 2 * AP_ROW_H), "Yaw   :");
    Lcd_Str(AP_X_UNIT, (uint16_t)(AP_Y_ROW0 + 2 * AP_ROW_H), " deg");

    Lcd_Str(0, (uint16_t)(AP_Y_ROW0 + 4 * AP_ROW_H), "q0");
    Lcd_Str(80, (uint16_t)(AP_Y_ROW0 + 4 * AP_ROW_H), "q1");
    Lcd_Str(0, (uint16_t)(AP_Y_ROW0 + 5 * AP_ROW_H), "q2");
    Lcd_Str(80, (uint16_t)(AP_Y_ROW0 + 5 * AP_ROW_H), "q3");

    Lcd_Str(0, AP_Y_NOTE,   "update 2ms/500Hz  X-Y-Z axis");
    Lcd_Str(0, AP_Y_FOOTER, "OK: RECALIBRATE   BACK:RET");
}

static void Att_Update(void)
{
    char buf[24];

    Lcd_BodyFont();

    // 三个欧拉角
    snprintf(buf, sizeof(buf), "%+7.2f", g_att.roll);
    Lcd_Str(AP_X_VAL, (uint16_t)(AP_Y_ROW0 + 0 * AP_ROW_H), buf);
    snprintf(buf, sizeof(buf), "%+7.2f", g_att.pitch);
    Lcd_Str(AP_X_VAL, (uint16_t)(AP_Y_ROW0 + 1 * AP_ROW_H), buf);
    snprintf(buf, sizeof(buf), "%+7.2f", g_att.yaw);
    Lcd_Str(AP_X_VAL, (uint16_t)(AP_Y_ROW0 + 2 * AP_ROW_H), buf);

    // 四元数。摆出来是为了说明真的是四元数在解算，不是三个角各积各的
    snprintf(buf, sizeof(buf), "%+.4f", g_att.q[0]);
    Lcd_Str(AP_X_Q0, (uint16_t)(AP_Y_ROW0 + 4 * AP_ROW_H), buf);
    snprintf(buf, sizeof(buf), "%+.4f", g_att.q[1]);
    Lcd_Str(AP_X_Q1, (uint16_t)(AP_Y_ROW0 + 4 * AP_ROW_H), buf);
    snprintf(buf, sizeof(buf), "%+.4f", g_att.q[2]);
    Lcd_Str(AP_X_Q0, (uint16_t)(AP_Y_ROW0 + 5 * AP_ROW_H), buf);
    snprintf(buf, sizeof(buf), "%+.4f", g_att.q[3]);
    Lcd_Str(AP_X_Q1, (uint16_t)(AP_Y_ROW0 + 5 * AP_ROW_H), buf);
}

// 重新做一次陀螺零偏校准，顺便把航向零点复位
static void Att_Calibrate(void)
{
    uint16_t i;

    NEUQ_Menu_BootScreen("GYRO CALIBRATING", "KEEP THE BOARD STILL");
    NEUQ_IMU_CalibrateGyro(1000);           // 500Hz下1000点就是2秒
    NEUQ_Attitude_SetGains(g_setting.att_kp, g_setting.att_ki);

    // 采一次真实数据，直接把四元数摆到当前姿态，不用等加速度计慢慢把它拉正
    NEUQ_IMU_Update();
    NEUQ_Attitude_InitFromAccel(g_imu.acc_g);

    // 再空跑100ms，让滤波器和姿态都进稳态
    for(i = 0; i < 50; i++)
    {
        NEUQ_IMU_Update();
        NEUQ_Attitude_Update(g_imu.acc_g, g_imu.gyro_dps,
                             (float)NEUQ_IMU_PERIOD_MS / 1000.0f);
        NEUQ_Delay_ms(NEUQ_IMU_PERIOD_MS);
    }

    NEUQ_Attitude_SetYawZero();             // 当前朝向记成航向0°
    s_page = MENU_PAGE_ATT;
    NEUQ_Menu_Init();
}

static void Att_HandleKey(KEY_EVENT_ENUM ev)
{
    switch(ev)
    {
        // 本页只有短按OK一个动作，含义唯一，不会误触发，就是重做一次零偏校准
        case KEY_FORWARD_SHORT:
            Att_Calibrate();
            break;

        case KEY_BACK_SHORT:
        case KEY_BACK_LONG:
            s_page = MENU_PAGE_MAIN;
            NEUQ_Menu_Init();
            break;

        default:
            break;
    }
}

/*==================== 对外接口 ====================*/
void NEUQ_Menu_Init(void)
{
    switch(s_page)
    {
        case MENU_PAGE_DATA:
            Data_DrawStatic();
            Data_Update();
            break;

        case MENU_PAGE_WAVE:
            Wave_DrawStatic();
            break;

        case MENU_PAGE_ATT:
            Att_DrawStatic();
            Att_Update();
            break;

        case MENU_PAGE_MAIN:
        default:
            Main_Draw();
            break;
    }
}

void NEUQ_Menu_HandleKey(KEY_EVENT_ENUM key_event)
{
    if(key_event == KEY_EVENT_NONE) return;

#if (NEUQ_MENU_KEY_DEBUG != 0)
    // 排错用，把每个按键事件和当前状态从UART_0(USART1 PA9)打出来。
    // 接个USB-TTL到PA9，串口助手115200，就能确认按键事件对不对、
    // 光标在第几项、有没有进编辑模式。平时把NEUQ_MENU_KEY_DEBUG改成0关掉。
    // 这里用Menu_DebugPrint不用printf，原因见它的注释。
    {
        char line[48];
        snprintf(line, sizeof(line), "[KEY] ev=%d page=%d cur=%d edit=%d\r\n",
                 (int)key_event, (int)s_page, (int)s_wave_cur, (int)s_editing);
        NEUQ_Menu_DebugPrint(line);
    }
#endif

    switch(s_page)
    {
        case MENU_PAGE_DATA: Data_HandleKey(key_event); break;
        case MENU_PAGE_WAVE: Wave_HandleKey(key_event); break;
        case MENU_PAGE_ATT:  Att_HandleKey(key_event);  break;
        case MENU_PAGE_MAIN:
        default:             Main_HandleKey(key_event); break;
    }
}

void NEUQ_Menu_Update(void)
{
    switch(s_page)
    {
        case MENU_PAGE_DATA:
            Data_Update();
            break;

        case MENU_PAGE_ATT:
            Att_Update();
            break;

        case MENU_PAGE_WAVE:
            // 状态区只在内容变了才重画，平时几乎不占刷屏时间
            if(strcmp(s_status_shown, s_status) != 0)
            {
                Wave_DrawStatus();
            }

            // Send行带实时发送计数，只在"这次显示的是ON"或者"刚从ON变成OFF"的时候重画。
            // 发送时每100ms刷这一行，23个字符大概2ms，占比很低，
            // 这样一眼就能看出串口有没有真的在往外发数据。
            if(g_setting.wave_enable || s_send_shown)
            {
                Wave_DrawRow(W_ITEM_SEND);
            }
            s_send_shown = g_setting.wave_enable;
            break;

        default:
            break;
    }
}

MENU_PAGE_ENUM NEUQ_Menu_GetPage(void)
{
    return s_page;
}

void NEUQ_Menu_SetStatus(const char *text)
{
    snprintf(s_status, sizeof(s_status), "%s", text);
}

void NEUQ_Menu_BootScreen(const char *line1, const char *line2)
{
    NEUQ_LCD200_Clear();

    Lcd_TitleFont();
    Lcd_Str(MP_X_LEFT, 70, "GYRO 6-AXIS SYSTEM");

    Lcd_BodyFont();
    Lcd_Str(MP_X_LEFT, 140, (char *)line1);
    Lcd_Str(MP_X_LEFT, 164, (char *)line2);
}
