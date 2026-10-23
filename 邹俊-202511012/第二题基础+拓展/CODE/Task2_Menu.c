/*********************************************************************************************************************
* 文件名称          Task2_Menu
* 功能说明          第二题：二级菜单（实现），说明见 Task2_Menu.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#include <stdio.h>
#include "Task_Config.h"
#include "Task2_Menu.h"

#if (TASK_SELECT == 2)

#include "My_Lcd.h"
#include "Task2_IMU.h"

// 页面
enum { PAGE_MAIN = 0, PAGE_DATA, PAGE_WAVE, PAGE_ATTITUDE, PAGE_NUM };
// Wave & Filter 页面的菜单项
enum { WAVE_AXIS = 0, WAVE_UPLOAD, WAVE_CUTOFF, WAVE_SAVE, WAVE_BACK };
// Attitude 页面的菜单项
enum { ATT_CALIB = 0, ATT_ZERO_YAW, ATT_BACK };

static const uint8_t     k_item_num[PAGE_NUM] = {3, 1, 5, 3};          // 每个页面有几个可选的菜单项
static const char *const k_axis_name[T2_AXIS_NUM] = {"AX", "AY", "AZ", "GX", "GY", "GZ"};

static uint8_t s_page = PAGE_MAIN;          // 当前页面
static uint8_t s_cursor[PAGE_NUM];          // 每个页面各自记住光标位置，返回后再进来光标还在原处
static uint8_t s_editing = 0;               // 1 表示正在编辑（Axis 或 Cutoff）
static uint8_t s_edit_axis = 0;             // 编辑中的候选值：按确认才真正写入后台，按返回就放弃
static float   s_edit_cutoff = 0.0f;
static uint8_t s_save_request = 0;          // 1 表示用户要求保存，等 Task2_Menu_poll 执行

static const char *axis_unit(uint8_t axis)
{
    return (axis < 3U) ? "g" : "dps";
}

void Task2_Menu_init(void)
{
    uint8_t i;
    s_page = PAGE_MAIN;
    for (i = 0; i < PAGE_NUM; i++) {
        s_cursor[i] = 0;
    }
    s_editing      = 0;
    s_save_request = 0;
    My_Lcd_invalidate();
}

//===================================================== 按键 =====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编辑状态下的按键
// 备注信息     上 / 下 改候选值（按住连续改）；确认 把候选值写进后台，立即生效；返回 放弃本次修改
//-------------------------------------------------------------------------------------------------------------------
static void menu_edit_key(const Key_Event_Struct *event)
{
    uint8_t item = s_cursor[PAGE_WAVE];

    if (event->id == KEY_UP) {
        if (item == WAVE_AXIS) {
            s_edit_axis++;                                  // 下一个轴，GZ 之后回到 AX
            if (s_edit_axis >= T2_AXIS_NUM) {
                s_edit_axis = 0;
            }
        }
        else {
            s_edit_cutoff += T2_CUTOFF_STEP_HZ;             // 截止频率加 0.5 Hz，不超过上限
            if (s_edit_cutoff > T2_CUTOFF_MAX_HZ) {
                s_edit_cutoff = T2_CUTOFF_MAX_HZ;
            }
        }
    }
    else if (event->id == KEY_DOWN) {
        if (item == WAVE_AXIS) {
            if (s_edit_axis == 0U) {                        // 上一个轴，AX 之前回到 GZ
                s_edit_axis = T2_AXIS_NUM - 1U;
            }
            else {
                s_edit_axis--;
            }
        }
        else {
            s_edit_cutoff -= T2_CUTOFF_STEP_HZ;             // 截止频率减 0.5 Hz，不低于下限
            if (s_edit_cutoff < T2_CUTOFF_MIN_HZ) {
                s_edit_cutoff = T2_CUTOFF_MIN_HZ;
            }
        }
    }
    else if (event->type == KEY_EVENT_PRESS && event->id == KEY_OK) {
        if (item == WAVE_AXIS) {
            g_t2.axis = s_edit_axis;
        }
        else {
            T2_Imu_set_cutoff(g_t2.axis, s_edit_cutoff);    // 改的是当前上传轴的截止频率
        }
        s_editing = 0;
    }
    else if (event->type == KEY_EVENT_PRESS && event->id == KEY_BACK) {
        s_editing = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     确认键：进入页面 / 执行光标所在的菜单项
//-------------------------------------------------------------------------------------------------------------------
static void menu_ok(void)
{
    uint8_t item = s_cursor[s_page];

    if (s_page == PAGE_MAIN) {
        s_page = (uint8_t)(PAGE_DATA + item);               // 三个入口依次对应三个二级页面
    }
    else if (s_page == PAGE_DATA) {
        s_page = PAGE_MAIN;                                 // 这一页只有 Back
    }
    else if (s_page == PAGE_WAVE) {
        if (item == WAVE_AXIS) {
            s_edit_axis = g_t2.axis;                        // 候选值从当前值开始
            s_editing   = 1;
        }
        else if (item == WAVE_UPLOAD) {
            g_t2.upload = (uint8_t)!g_t2.upload;            // 开 / 关上传，立即生效
        }
        else if (item == WAVE_CUTOFF) {
            s_edit_cutoff = g_t2.cutoff_hz[g_t2.axis];
            s_editing     = 1;
        }
        else if (item == WAVE_SAVE) {
            s_save_request = 1;                             // 先让屏幕显示 saving...，再在 Task2_Menu_poll 里真正写 Flash
        }
        else {
            s_page = PAGE_MAIN;
        }
    }
    else {                                                  // PAGE_ATTITUDE
        if (item == ATT_CALIB) {
            g_t2.calib_request = 1;                         // 交给中断去做，中断里会清零这个请求
        }
        else if (item == ATT_ZERO_YAW) {
            g_t2.zero_yaw_request = 1;
        }
        else {
            s_page = PAGE_MAIN;
        }
    }
}

uint8_t Task2_Menu_handle_key(const Key_Event_Struct *event)
{
    uint8_t count = k_item_num[s_page];

    if (s_editing) {
        menu_edit_key(event);
    }
    else if (event->id == KEY_UP) {
        if (s_cursor[s_page] == 0U) {                       // 光标上移，到顶回到最后一项
            s_cursor[s_page] = (uint8_t)(count - 1U);
        }
        else {
            s_cursor[s_page]--;
        }
    }
    else if (event->id == KEY_DOWN) {
        s_cursor[s_page]++;                                 // 光标下移，到底回到第一项
        if (s_cursor[s_page] >= count) {
            s_cursor[s_page] = 0;
        }
    }
    else if (event->type == KEY_EVENT_PRESS && event->id == KEY_OK) {
        menu_ok();
    }
    else if (event->type == KEY_EVENT_PRESS && event->id == KEY_BACK) {
        s_page = PAGE_MAIN;
    }
    return 1;
}

uint8_t Task2_Menu_poll(void)
{
    if (!s_save_request) {
        return 0;
    }
    T2_Imu_save_param();                                    // 结果记录在 g_t2 里，状态行会显示
    s_save_request = 0;
    return 1;
}

//===================================================== 绘制 =====================================================
// 画一个菜单项：光标所在行蓝底，前面加 "> "
static void draw_item(uint8_t row, uint8_t selected, const char *text)
{
    My_Lcd_print(row, (UI_Style_Enum)(selected ? UI_STYLE_SELECT : UI_STYLE_NORMAL), "%s%s", selected ? "> " : "  ", text);
}

// 传感器 / 校准状态行
static void draw_sensor_status(uint8_t row)
{
    if (!g_t2.sensor_ok) {
        My_Lcd_print(row, UI_STYLE_WARN, "MPU6050 NOT FOUND, retrying...");
    }
    else if (g_t2_calib.state == T2_CALIB_RUNNING) {
        My_Lcd_print(row, UI_STYLE_WARN, "Gyro calib %3u%%  keep still!",
                      (unsigned int)(g_t2_calib.count * 100U / (T2_CALIB_TIME_MS / T2_SAMPLE_PERIOD_MS)));
    }
    else {
        My_Lcd_print(row, UI_STYLE_GOOD, "MPU6050 OK %uHz n=%lu",
                      (unsigned int)(1000U / T2_SAMPLE_PERIOD_MS), (unsigned long)g_t2.sample_count);
    }
}

// Flash 状态行
static void draw_flash_status(uint8_t row)
{
    if (s_save_request)               My_Lcd_print(row, UI_STYLE_WARN,   "Flash: saving...");
    else if (!g_t2.flash_ok)          My_Lcd_print(row, UI_STYLE_WARN,   "Flash: NOT FOUND (defaults)");
    else if (g_t2.save_failed)        My_Lcd_print(row, UI_STYLE_WARN,   "Flash: SAVE FAILED");
    else if (g_t2.param_dirty)        My_Lcd_print(row, UI_STYLE_WARN,   "Flash: unsaved changes *");
    else if (g_t2.flash_has_record)   My_Lcd_print(row, UI_STYLE_GOOD,   "Flash: saved");
    else                              My_Lcd_print(row, UI_STYLE_NORMAL, "Flash: no record, defaults");
}

// 六个轴的截止频率，依次为 AX AY AZ GX GY GZ
static void draw_cutoffs(uint8_t row)
{
    My_Lcd_print(row, UI_STYLE_NORMAL, "Fc %4.1f %4.1f %4.1f %4.1f %4.1f %4.1f",
                  (double)g_t2.cutoff_hz[0], (double)g_t2.cutoff_hz[1], (double)g_t2.cutoff_hz[2],
                  (double)g_t2.cutoff_hz[3], (double)g_t2.cutoff_hz[4], (double)g_t2.cutoff_hz[5]);
}

// 一级菜单
static void draw_main(void)
{
    uint8_t cursor = s_cursor[PAGE_MAIN];

    My_Lcd_print(0, UI_STYLE_TITLE, " Task2  MPU6050   Main Menu");
    draw_sensor_status(1);
    draw_item(2, cursor == 0U, "1. Six-Axis Data");
    draw_item(3, cursor == 1U, "2. Wave & Filter (VOFA+)");
    draw_item(4, cursor == 2U, "3. Attitude (Quaternion)");
    My_Lcd_print(5, UI_STYLE_NORMAL, "%s", "");
    My_Lcd_print(6, UI_STYLE_NORMAL, "Upload %-3s  Axis %s (%s)",          // 一级页面也显示后台设置，方便检查
                  g_t2.upload ? "ON" : "OFF", k_axis_name[g_t2.axis], axis_unit(g_t2.axis));
    draw_cutoffs(7);
    draw_flash_status(8);
    My_Lcd_print(9, UI_STYLE_HINT, "UP/DN:move  OK:enter");
}

// 六轴数据：物理量 + 原始计数
static void draw_data(void)
{
    uint8_t i;

    My_Lcd_print(0, UI_STYLE_TITLE, " Six-Axis Data   value / raw");
    for (i = 0; i < T2_AXIS_NUM; i++) {
        if (i < 3U) {
            My_Lcd_print((uint8_t)(1U + i), UI_STYLE_NORMAL, "%s %+9.4f g     raw %+6d",
                          k_axis_name[i], (double)g_t2.value[i], (int)g_t2.raw[i]);
        }
        else {
            My_Lcd_print((uint8_t)(1U + i), UI_STYLE_NORMAL, "%s %+9.2f dps   raw %+6d",
                          k_axis_name[i], (double)g_t2.value[i], (int)g_t2.raw[i]);
        }
    }
    if (g_t2.sensor_ok) {
        My_Lcd_print(7, UI_STYLE_NORMAL, "Temp %5.1f C   n=%lu", (double)g_t2.temperature, (unsigned long)g_t2.sample_count);
    }
    else {
        My_Lcd_print(7, UI_STYLE_WARN, "MPU6050 NOT FOUND, retrying...");
    }
    My_Lcd_print(8, UI_STYLE_HINT, "+-8g 4096/g  +-2000dps 16.4/dps");    // 量程与"原始计数 → 物理量"的换算比例
    draw_item(9, 1, "Back");
}

// 波形与滤波设置
static void draw_wave(void)
{
    uint8_t axis   = g_t2.axis;
    uint8_t cursor = s_cursor[PAGE_WAVE];
    char    text[UI_COLS + 1];

    My_Lcd_print(0, UI_STYLE_TITLE, " Wave & Filter   -> VOFA+");

    if (s_editing && cursor == WAVE_AXIS) {         // 编辑中：黄底显示候选值
        My_Lcd_print(1, UI_STYLE_EDIT, "  Axis   : < %s > (%s)", k_axis_name[s_edit_axis], axis_unit(s_edit_axis));
    }
    else {
        snprintf(text, sizeof(text), "Axis   : %s (%s)", k_axis_name[axis], axis_unit(axis));
        draw_item(1, cursor == WAVE_AXIS, text);
    }

    snprintf(text, sizeof(text), "Upload : %s", g_t2.upload ? "ON" : "OFF");
    draw_item(2, cursor == WAVE_UPLOAD, text);

    if (s_editing && cursor == WAVE_CUTOFF) {
        My_Lcd_print(3, UI_STYLE_EDIT, "  Cutoff : < %4.1f > Hz [%s]", (double)s_edit_cutoff, k_axis_name[axis]);
    }
    else {
        snprintf(text, sizeof(text), "Cutoff : %4.1f Hz  [%s]", (double)g_t2.cutoff_hz[axis], k_axis_name[axis]);
        draw_item(3, cursor == WAVE_CUTOFF, text);
    }

    draw_item(4, cursor == WAVE_SAVE, "Save params to Flash");
    draw_item(5, cursor == WAVE_BACK, "Back");
    My_Lcd_print(6, UI_STYLE_NORMAL, "%s raw %+9.3f filt %+9.3f",          // 屏幕上也能直接看到滤波效果
                  k_axis_name[axis], (double)g_t2.value[axis], (double)g_t2.filtered[axis]);
    draw_cutoffs(7);
    draw_flash_status(8);
    if (s_editing) {
        My_Lcd_print(9, UI_STYLE_HINT, "UP:+ DN:- OK:apply BACK:cancel");
    }
    else {
        My_Lcd_print(9, UI_STYLE_HINT, "UP/DN:move OK:select BACK:ret");
    }
}

// 姿态解算
static void draw_attitude(void)
{
    uint8_t cursor = s_cursor[PAGE_ATTITUDE];

    My_Lcd_print(0, UI_STYLE_TITLE, " Attitude  (Mahony quaternion)");
    if (g_t2_calib.state == T2_CALIB_DONE) {
        My_Lcd_print(1, UI_STYLE_NORMAL, "Roll  (X) %+8.2f deg", (double)g_t2_attitude.roll);
        My_Lcd_print(2, UI_STYLE_NORMAL, "Pitch (Y) %+8.2f deg", (double)g_t2_attitude.pitch);
        My_Lcd_print(3, UI_STYLE_NORMAL, "Yaw   (Z) %+8.2f deg (rel)", (double)g_t2_attitude.yaw);
    }
    else {                                          // 还没校准完，姿态还没开始解算
        My_Lcd_print(1, UI_STYLE_NORMAL, "Roll  (X)     ---- deg");
        My_Lcd_print(2, UI_STYLE_NORMAL, "Pitch (Y)     ---- deg");
        My_Lcd_print(3, UI_STYLE_NORMAL, "Yaw   (Z)     ---- deg (rel)");
    }
    My_Lcd_print(4, UI_STYLE_NORMAL, "q %+6.3f %+6.3f %+6.3f %+6.3f",
                  (double)g_t2_attitude.q0, (double)g_t2_attitude.q1, (double)g_t2_attitude.q2, (double)g_t2_attitude.q3);

    if (!g_t2.sensor_ok) {
        My_Lcd_print(5, UI_STYLE_WARN, "MPU6050 NOT FOUND, retrying...");
    }
    else if (g_t2_calib.state == T2_CALIB_RUNNING) {
        My_Lcd_print(5, UI_STYLE_WARN, "Calib %3u%% keep still! (x%u)",   // x 后面是因为被碰到而重来的次数
                      (unsigned int)(g_t2_calib.count * 100U / (T2_CALIB_TIME_MS / T2_SAMPLE_PERIOD_MS)),
                      (unsigned int)g_t2_calib.restarts);
    }
    else if (g_t2_calib.state == T2_CALIB_DONE) {
        My_Lcd_print(5, UI_STYLE_GOOD, "Bias %+5.2f %+5.2f %+5.2f dps",
                      (double)g_t2_calib.gyro_bias[0], (double)g_t2_calib.gyro_bias[1], (double)g_t2_calib.gyro_bias[2]);
    }
    else {
        My_Lcd_print(5, UI_STYLE_WARN, "Not calibrated");
    }
    My_Lcd_print(6, UI_STYLE_HINT, "X fwd Y left Z up | dt %u ms", (unsigned int)T2_SAMPLE_PERIOD_MS);
    draw_item(7, cursor == ATT_CALIB,    "Recalibrate gyro (keep still)");
    draw_item(8, cursor == ATT_ZERO_YAW, "Zero yaw (set heading = 0)");
    draw_item(9, cursor == ATT_BACK,     "Back");
}

void Task2_Menu_draw(void)
{
    if (s_page == PAGE_DATA) {
        draw_data();
    }
    else if (s_page == PAGE_WAVE) {
        draw_wave();
    }
    else if (s_page == PAGE_ATTITUDE) {
        draw_attitude();
    }
    else {
        draw_main();
    }
}

#endif
