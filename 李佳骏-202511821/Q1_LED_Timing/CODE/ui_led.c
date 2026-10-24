/* ui_led.c - 第一题页面绘制 */

#include "ui_led.h"
#include "board_config.h"
#include "led_pattern.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

/* 横屏 320×240（库内 LCD200_DIR_Cross），所有 y 坐标必须 < 240 */
#define UI_TITLE_X      (8)
#define UI_STATUS_X     (8)
#define UI_PENDING_X    (170)
#define UI_LIST_X       (8)
#define UI_FOOTER_X     (8)

/* 调试模式：在 Current 下显示四键电平 */
#if (T1_SHOW_KEY_LEVELS != 0)
#define UI_TITLE_Y      (4)
#define UI_CURRENT_Y    (32)
#define UI_DEBUG_Y      (58)
#define UI_VOLTAGE_Y    (80)
#define UI_LIST_HINT_Y  (108)
#define UI_LIST_Y0      (126)
#define UI_LIST_DY      (19)
#define UI_FOOTER_Y     (208)
#else
#define UI_TITLE_Y      (4)
#define UI_CURRENT_Y    (32)
#define UI_VOLTAGE_Y    (62)
#define UI_LIST_HINT_Y  (92)
#define UI_LIST_Y0      (112)
#define UI_LIST_DY      (20)
#define UI_FOOTER_Y     (204)
#endif

void Ui_Led_DrawAll(void)
{
    uint8_t current = LedPattern_GetCurrent();
    uint8_t pending = LedPattern_GetPending();
    uint8_t i;

    NEUQ_LCD200_Clear();

    /* 标题与状态 */
    NEUQ_LCD200_SetTextFont(&ASCII_Font24);
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    NEUQ_LCD200_SetBackColor(LCD_BLACK);

    NEUQ_LCD200_DisplayString(UI_TITLE_X, UI_TITLE_Y, "LED TIMING");

    NEUQ_LCD200_DisplayString(UI_STATUS_X, UI_CURRENT_Y, "Current:");
    NEUQ_LCD200_Display_int32(UI_STATUS_X + 108, UI_CURRENT_Y, (int32_t)(current + 1U), 1);

    /* 横屏之后宽度有 320，「待执行方案」可以和「当前方案」并排放，省下一行 */
    NEUQ_LCD200_DisplayString(UI_PENDING_X, UI_CURRENT_Y, "Pending:");
    NEUQ_LCD200_Display_int32(UI_PENDING_X + 108, UI_CURRENT_Y, (int32_t)(pending + 1U), 1);

    NEUQ_LCD200_DisplayString(UI_STATUS_X, UI_VOLTAGE_Y, "Voltage: --");

    /* 四套方案一览（小一号字体） */
    NEUQ_LCD200_SetTextFont(&ASCII_Font16);

    NEUQ_LCD200_SetPenColor(LCD_GREY);
    NEUQ_LCD200_DisplayString(UI_LIST_X, UI_LIST_HINT_Y, "ON/OFF seq (ms)");
    NEUQ_LCD200_SetPenColor(LCD_WHITE);

    for (i = 0; i < T1_PATTERN_COUNT; i++)
    {
        char        line[32];
        const char *desc;
        uint8_t     n = 0;

        /* 前缀两个字符：前者标记当前执行，后者标记待执行 */
        line[0] = (i == current) ? '>' : ' ';
        line[1] = ((i == pending) && (pending != current)) ? '*' : ' ';
        line[2] = (char)('1' + i);
        line[3] = ':';
        line[4] = ' ';

        desc = LedPattern_GetSummary(i);
        while (desc[n] != '\0' && n < (sizeof(line) - 6U))
        {
            line[5 + n] = desc[n];
            n++;
        }
        line[5 + n] = '\0';

        NEUQ_LCD200_DisplayString(UI_LIST_X, UI_LIST_Y0 + i * UI_LIST_DY, line);
    }

    /* 底部操作提示 */
    NEUQ_LCD200_SetPenColor(LCD_GREY);
    NEUQ_LCD200_DisplayString(UI_FOOTER_X, UI_FOOTER_Y, "SW3:Next  SW5:Prev");
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
}

void Ui_Led_RefreshKeyLevels(void)
{
#if (T1_SHOW_KEY_LEVELS != 0)
    static const GPIO_Ports_ENUM pins[4] = { PE9, PE8, PE7, PE10 };
    static const char *const     name[4] = { "PE9", "PE8", "PE7", "PE10" };
    char    line[28];
    uint8_t i;
    uint8_t n = 0;

    for (i = 0; i < 4U; i++)
    {
        uint8_t k = 0;

        if (i > 0U)
        {
            line[n++] = ' ';
        }
        while (name[i][k] != '\0' && n < (sizeof(line) - 3U))
        {
            line[n++] = name[i][k];
            k++;
        }
        line[n++] = '=';
        line[n++] = (NEUQ_GPIO_read(pins[i]) == GPIO_Low) ? '0' : '1';
    }
    line[n] = '\0';

    NEUQ_LCD200_SetTextFont(&ASCII_Font16);
    NEUQ_LCD200_ClearRect(0, (uint16_t)(UI_DEBUG_Y - 2), 240, 20);
    NEUQ_LCD200_SetPenColor(LCD_YELLOW);
    NEUQ_LCD200_DisplayString(4, UI_DEBUG_Y, line);
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
#else
    /* 开关关掉时不产生任何代码 */
#endif
}
