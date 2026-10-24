/* ui_imu.c - 第二题页面绘制 */

#include "ui_imu.h"
#include "board_config.h"
#include "menu.h"
#include "imu_filter.h"
#include "imu_attitude.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

/* 横屏 320×240（库内 LCD200_DIR_Cross），所有 y 坐标必须 < 240 */
#define UI_CHAR_W       (12U)

/* 六轴页坐标：320 宽够放两列，六个轴排成 3 行 × 2 列，温度单独一行 */
#define SIX_LABEL_X     (8U)
#define SIX_VALUE_X     (46U)
#define SIX_UNIT_X      (134U)
#define SIX_COL_DX      (162U)      /* 第二列相对第一列的横向偏移 */
#define SIX_Y0          (40U)
#define SIX_ROW_H       (30U)
#define SIX_VALUE_LEN   (7U)
#define SIX_FOOTER_Y    (176U)

/* 波形页坐标 */
#define WAVE_ITEM_X     (8U)
#define WAVE_Y0         (38U)
#define WAVE_ITEM_DY    (30U)
#define WAVE_VALUE_X    (160U)
#define WAVE_RAW_Y      (168U)

static void Ui_DrawFloatFixed(uint16_t x, uint16_t y, float value, uint8_t len, uint8_t decimal)
{
    NEUQ_LCD200_ClearRect(x, y, (uint16_t)(len * UI_CHAR_W), 24U);
    NEUQ_LCD200_DisplayFloat(x, y, value, len, decimal);
}

static void Ui_DrawIntFixed(uint16_t x, uint16_t y, int32_t value, uint8_t len)
{
    NEUQ_LCD200_ClearRect(x, y, (uint16_t)(len * UI_CHAR_W), 24U);
    NEUQ_LCD200_Display_int32(x, y, value, len);
}

static void Ui_DrawMainPage(const imu_view_t *view)
{
    uint8_t cursor = Menu_GetCursor();
    uint8_t i;

    static const char *const items[3] = {
        "Six-axis data",
        "Wave & filter",
        "Attitude solve",
    };

    NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    NEUQ_LCD200_DisplayString(8, 4, "IMU MENU");

    for (i = 0; i < 3U; i++)
    {
        char line[24];

        line[0] = (i == cursor) ? '>' : ' ';
        line[1] = ' ';
        line[2] = (char)('1' + i);
        line[3] = '.';
        line[4] = ' ';

        {
            uint8_t n = 0;
            while (items[i][n] != '\0' && n < 18U)
            {
                line[5 + n] = items[i][n];
                n++;
            }
            line[5 + n] = '\0';
        }

        NEUQ_LCD200_DisplayString(WAVE_ITEM_X, (uint16_t)(40U + i * 34U), line);
    }

    /* 传感器状态：方便上机时一眼看出模块有没有认到 */
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_GREY);
    if (view->imu_ok != 0U)
    {
        char buf[20];
        buf[0] = 'I'; buf[1] = 'M'; buf[2] = 'U'; buf[3] = ':';
        buf[4] = ' '; buf[5] = '0'; buf[6] = 'x';
        buf[7] = "0123456789ABCDEF"[(view->imu_addr7 >> 4) & 0x0F];
        buf[8] = "0123456789ABCDEF"[view->imu_addr7 & 0x0F];
        buf[9] = '\0';
        NEUQ_LCD200_DisplayString(8, 150, buf);
    }
    else
    {
        NEUQ_LCD200_DisplayString(8, 150, "IMU: not found");
    }
    NEUQ_LCD200_DisplayString(8, 182, "SW5:up SW3:down  SW8:ok SW9:back");
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
}

static void Ui_DrawAxisRow(imu_axis_t axis, float value, uint16_t x, uint16_t y)
{
    uint16_t value_x = (uint16_t)(x + (SIX_VALUE_X - SIX_LABEL_X));
    uint16_t unit_x  = (uint16_t)(x + (SIX_UNIT_X  - SIX_LABEL_X));
    uint8_t  decimal = (axis <= IMU_AZ) ? 3U : 1U;

    NEUQ_LCD200_DisplayString(x, y, (char *)IMU_AxisName(axis));
    Ui_DrawFloatFixed(value_x, y, value, SIX_VALUE_LEN, decimal);
    NEUQ_LCD200_DisplayString(unit_x, y, (char *)IMU_AxisUnit(axis));
}

static void Ui_DrawSixAxisPage(const imu_view_t *view)
{
    uint8_t  i;
    uint16_t y;

    NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    NEUQ_LCD200_DisplayString(8, 4, "SIX AXIS");

    /* 左列三行加速度，右列三行角速度，正好把 320 的宽度用起来 */
    for (i = 0; i < 3U; i++)
    {
        y = (uint16_t)(SIX_Y0 + i * SIX_ROW_H);
        Ui_DrawAxisRow((imu_axis_t)i, view->raw.acc[i], SIX_LABEL_X, y);
        Ui_DrawAxisRow((imu_axis_t)(i + 3U), view->raw.gyro[i],
                       (uint16_t)(SIX_LABEL_X + SIX_COL_DX), y);
    }

    y = (uint16_t)(SIX_Y0 + 3U * SIX_ROW_H);
    NEUQ_LCD200_DisplayString(SIX_LABEL_X, y, "T");
    Ui_DrawFloatFixed((uint16_t)(SIX_LABEL_X + 38U), y, view->raw.temp_c, 6U, 1U);
    NEUQ_LCD200_DisplayString((uint16_t)(SIX_LABEL_X + 114U), y, "C");

    /* 底部一行：当前轴的滤波系数与上传状态，便于和 VOFA+ 对照 */
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_GREY);
    NEUQ_LCD200_DisplayString(8, SIX_FOOTER_Y, "Alpha:");
    NEUQ_LCD200_ClearRect(64, SIX_FOOTER_Y, 48, 16);
    NEUQ_LCD200_DisplayFloat(64, SIX_FOOTER_Y, ImuFilter_GetAlpha(Menu_GetAxis()), 4U, 2U);
    NEUQ_LCD200_DisplayString(140, SIX_FOOTER_Y,
                              Menu_IsUploadOn() ? "Upload: ON " : "Upload: OFF");
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
}

static void Ui_DrawWavePage(const imu_view_t *view)
{
    uint8_t        cursor = Menu_GetCursor();
    uint8_t        editing = Menu_IsEditing();
    imu_axis_t     axis = Menu_GetAxis();
    uint16_t       y;
    float          shown_alpha;
    static const char *const item_name[4] = { "Axis", "Upload", "Alpha", "Save" };
    uint8_t        i;

    NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    NEUQ_LCD200_DisplayString(8, 4, "WAVE & FILTER");

    for (i = 0; i < MENU_WAVE_ITEM_COUNT; i++)
    {
        char line[26];
        uint8_t n = 0;

        /* 编辑中的那一行用 '*' 标记，普通选中行用 '>' */
        if ((i == cursor) && (editing != 0U) && (i == MENU_WAVE_ITEM_ALPHA))
        {
            line[0] = '*';
        }
        else
        {
            line[0] = (i == cursor) ? '>' : ' ';
        }
        line[1] = ' ';

        while (item_name[i][n] != '\0' && n < 8U)
        {
            line[2 + n] = item_name[i][n];
            n++;
        }
        while (n < 8U)
        {
            line[2 + n] = ' ';
            n++;
        }
        line[10] = ':';
        line[11] = ' ';
        line[12] = '\0';

        y = (uint16_t)(WAVE_Y0 + i * WAVE_ITEM_DY);
        NEUQ_LCD200_DisplayString(WAVE_ITEM_X, y, line);

        switch (i)
        {
            case MENU_WAVE_ITEM_AXIS:
                NEUQ_LCD200_DisplayString(WAVE_VALUE_X, y, (char *)IMU_AxisName(axis));
                break;

            case MENU_WAVE_ITEM_UPLOAD:
                NEUQ_LCD200_DisplayString(WAVE_VALUE_X, y, Menu_IsUploadOn() ? "ON " : "OFF");
                break;

            case MENU_WAVE_ITEM_ALPHA:
                shown_alpha = (editing != 0U) ? Menu_GetEditValue()
                                              : ImuFilter_GetAlpha(axis);
                Ui_DrawFloatFixed(WAVE_VALUE_X, y, shown_alpha, 4U, 2U);
                break;

            case MENU_WAVE_ITEM_SAVE:
                NEUQ_LCD200_DisplayString(WAVE_VALUE_X, y, "to flash");
                break;

            default:
                break;
        }
    }

    /* 当前轴的原始值与滤波值，方便和 VOFA+ 上看到的曲线对照 */
    y = WAVE_RAW_Y;
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_DisplayString(8, y, "raw :");
    NEUQ_LCD200_DisplayString(8, (uint16_t)(y + 24U), "filt:");
    NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    Ui_DrawFloatFixed(88, y, IMU_AxisValue(&view->raw, axis), 8U, 3U);
    Ui_DrawFloatFixed(88, (uint16_t)(y + 24U), view->filtered[axis], 8U, 3U);
}

static void Ui_RefreshAttitude(void)
{
    const attitude_t *att = Attitude_Get();

    /* 三个角：只有在校准完成后才有意义，校准期间显示 "--" */
    NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    if (att->state == ATT_STATE_READY)
    {
        Ui_DrawFloatFixed(130, 38,  att->roll,  6U, 1U);
        Ui_DrawFloatFixed(130, 70,  att->pitch, 6U, 1U);
        Ui_DrawFloatFixed(130, 102, att->yaw,   6U, 1U);
    }
    else
    {
        NEUQ_LCD200_ClearRect(130, 38,  72, 24);
        NEUQ_LCD200_ClearRect(130, 70,  72, 24);
        NEUQ_LCD200_ClearRect(130, 102, 72, 24);
        NEUQ_LCD200_DisplayString(130, 38,  "--");
        NEUQ_LCD200_DisplayString(130, 70,  "--");
        NEUQ_LCD200_DisplayString(130, 102, "--");
    }

    /* 状态行 */
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_ClearRect(0, 140, 320, 18);
    if (att->state == ATT_STATE_WAIT)
    {
        NEUQ_LCD200_SetPenColor(LCD_YELLOW);
        NEUQ_LCD200_DisplayString(8, 141, "Status: get ready, keep still");
    }
    else if (att->state == ATT_STATE_SAMPLING)
    {
        NEUQ_LCD200_SetPenColor(LCD_YELLOW);
        NEUQ_LCD200_DisplayString(8, 141, "Status: calibrating");
        NEUQ_LCD200_Display_int32(200, 141, (int32_t)att->progress, 3);
        NEUQ_LCD200_DisplayString(232, 141, "%");
    }
    else
    {
        NEUQ_LCD200_SetPenColor(LCD_GREEN);
        NEUQ_LCD200_DisplayString(8, 141, "Status: ready");
    }
    NEUQ_LCD200_SetPenColor(LCD_WHITE);

    /* 实测更新周期：验收时被问到直接指这一行 */
    NEUQ_LCD200_ClearRect(0, 164, 320, 18);
    NEUQ_LCD200_SetPenColor(LCD_GREY);
    NEUQ_LCD200_DisplayString(8, 165, "Period:");
    NEUQ_LCD200_Display_int32(72, 165, (int32_t)att->period_ms, 3);
    NEUQ_LCD200_DisplayString(104, 165, "ms");
    NEUQ_LCD200_SetPenColor(LCD_WHITE);

    /* 校准期间的醒目提示 */
    NEUQ_LCD200_ClearRect(0, 190, 320, 18);
    if (att->state != ATT_STATE_READY)
    {
        NEUQ_LCD200_SetPenColor(LCD_RED);
        NEUQ_LCD200_DisplayString(8, 191, "Do not move the board!");
        NEUQ_LCD200_SetPenColor(LCD_WHITE);
    }
}

static void Ui_DrawAttitudePage(void)
{
    NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    NEUQ_LCD200_DisplayString(8, 4, "ATTITUDE");

    NEUQ_LCD200_DisplayString(8, 38,  "Roll :");
    NEUQ_LCD200_DisplayString(8, 70,  "Pitch:");
    NEUQ_LCD200_DisplayString(8, 102, "Yaw  :");

    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_GREY);
    NEUQ_LCD200_DisplayString(8, 215, "SW8: Recalibrate");
    NEUQ_LCD200_SetPenColor(LCD_WHITE);

    Ui_RefreshAttitude();
}

void Ui_Imu_DrawPage(const imu_view_t *view)
{
    NEUQ_LCD200_Clear();

    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    NEUQ_LCD200_SetBackColor(LCD_BLACK);

    switch (Menu_GetPage())
    {
        case MENU_PAGE_MAIN:     Ui_DrawMainPage(view);      break;
        case MENU_PAGE_SIXAXIS:  Ui_DrawSixAxisPage(view);   break;
        case MENU_PAGE_WAVE:     Ui_DrawWavePage(view);      break;
        default:                 Ui_DrawAttitudePage();      break;
    }
}

void Ui_Imu_RefreshValues(const imu_view_t *view)
{
    uint8_t i;

    if (Menu_GetPage() == MENU_PAGE_SIXAXIS)
    {
        uint16_t ty;

        /* 左列加速度、右列角速度，各三行，和整页重画时的位置保持一致 */
        for (i = 0; i < 3U; i++)
        {
            ty = (uint16_t)(SIX_Y0 + i * SIX_ROW_H);
            Ui_DrawFloatFixed(SIX_VALUE_X, ty, view->raw.acc[i], SIX_VALUE_LEN, 3U);
            Ui_DrawFloatFixed((uint16_t)(SIX_VALUE_X + SIX_COL_DX), ty,
                              view->raw.gyro[i], SIX_VALUE_LEN, 1U);
        }

        ty = (uint16_t)(SIX_Y0 + 3U * SIX_ROW_H);
        Ui_DrawFloatFixed((uint16_t)(SIX_LABEL_X + 38U), ty,
                          view->raw.temp_c, 6U, 1U);

        /* 底部状态行的滤波系数 */
        NEUQ_LCD200_SetTextFont(&ASCII_Font16);
        NEUQ_LCD200_ClearRect(64, SIX_FOOTER_Y, 48, 16);
        NEUQ_LCD200_DisplayFloat(64, SIX_FOOTER_Y,
                                 ImuFilter_GetAlpha(Menu_GetAxis()), 4U, 2U);
        NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    }
    else if (Menu_GetPage() == MENU_PAGE_WAVE)
    {
        imu_axis_t axis = Menu_GetAxis();
        uint16_t   y    = (uint16_t)(WAVE_Y0 + MENU_WAVE_ITEM_ALPHA * WAVE_ITEM_DY);

        /* 编辑中用临时值，否则显示当前真实生效的系数 */
        if (Menu_IsEditing() != 0U)
        {
            Ui_DrawFloatFixed(WAVE_VALUE_X, y, Menu_GetEditValue(), 4U, 2U);
        }
        else
        {
            Ui_DrawFloatFixed(WAVE_VALUE_X, y, ImuFilter_GetAlpha(axis), 4U, 2U);
        }

        Ui_DrawFloatFixed(88, WAVE_RAW_Y, IMU_AxisValue(&view->raw, axis), 8U, 3U);
        Ui_DrawFloatFixed(88, (uint16_t)(WAVE_RAW_Y + 24U), view->filtered[axis], 8U, 3U);
    }
    else if (Menu_GetPage() == MENU_PAGE_ATTITUDE)
    {
        /* 姿态页的三个角、状态、周期都在变，整体刷新 */
        Ui_RefreshAttitude();
    }
}

void Ui_Imu_ShowSaved(void)
{
    NEUQ_LCD200_ClearRect(0, 212, 320, 20);
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_GREEN);
    NEUQ_LCD200_DisplayString(8, 214, "Saved to flash.");
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
}
