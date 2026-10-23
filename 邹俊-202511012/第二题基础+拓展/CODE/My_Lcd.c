/*********************************************************************************************************************
* 文件名称          My_Lcd
* 功能说明          按行显示文字（实现），说明见 My_Lcd.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include <stdarg.h>
#include "My_Lcd.h"

// 每种样式的字色和底色（RGB888，库会自动转成屏幕用的 RGB565），顺序与 UI_Style_Enum 一致
static const uint32_t s_pen_color[UI_STYLE_NUM]  = {LCD_BLACK, LCD_WHITE, LCD_WHITE, LCD_BLACK, DARK_GREEN, LCD_RED, DARK_GREY};
static const uint32_t s_back_color[UI_STYLE_NUM] = {LCD_WHITE, DARK_BLUE, LCD_BLUE,  LCD_YELLOW, LCD_WHITE, LCD_WHITE, LCD_WHITE};

// 屏幕内容缓存：每一行现在显示的文字和样式
static char    s_text[UI_ROWS][UI_COLS + 1];
static uint8_t s_style[UI_ROWS];
static uint8_t s_valid[UI_ROWS];        // 0 表示这一行的缓存不可信（刚上电 / 被清过），下次必须整行重画

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化屏幕
// 备注信息     NEUQ_LCD200_init 内部会调用 printf 打印调试信息，所以之前必须先 NEUQ_Debug_init 初始化串口，
//              否则 printf 会一直等待一个没打开的串口
//-------------------------------------------------------------------------------------------------------------------
void My_Lcd_init(void)
{
    NEUQ_LCD200_init();                         // 横屏、白底清屏、打开背光
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font20);    // 10x20 字体，一行正好 32 个字
    My_Lcd_invalidate();
}

void My_Lcd_invalidate(void)
{
    uint8_t row;
    for (row = 0; row < UI_ROWS; row++) {
        s_valid[row] = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示一整行
// 参数说明     row     行号 0 ~ UI_ROWS-1
// 参数说明     style   样式，见 UI_Style_Enum
// 参数说明     fmt     和 printf 一样的格式字符串
// 备注信息     1. 先把内容格式化成正好 32 个字符（不足补空格），这样新内容一定能完全覆盖旧内容
//              2. 样式变了（比如光标移到这一行）就整行重画；样式没变就逐个字符比较，只画变了的字符
//-------------------------------------------------------------------------------------------------------------------
void My_Lcd_print(uint8_t row, UI_Style_Enum style, const char *fmt, ...)
{
    char    line[UI_COLS + 1];
    va_list args;
    int     len;
    uint8_t col;
    uint16_t y = (uint16_t)(row * UI_ROW_PITCH);

    if (row >= UI_ROWS || style >= UI_STYLE_NUM) {
        return;
    }

    // 1. 格式化并补齐到 32 个字符；字库里只有可见 ASCII 字符，其他字符一律换成空格
    va_start(args, fmt);
    len = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    for (col = 0; col < UI_COLS; col++) {
        if ((int)col >= len || (uint8_t)line[col] < 32U || (uint8_t)line[col] > 126U) {
            line[col] = ' ';
        }
    }
    line[UI_COLS] = '\0';

    NEUQ_LCD200_SetPenColor(s_pen_color[style]);
    NEUQ_LCD200_SetBackColor(s_back_color[style]);

    // 2. 样式变了或缓存不可信：整行重画
    if (!s_valid[row] || s_style[row] != (uint8_t)style) {
        NEUQ_LCD200_DisplayString(0, y, line);
        memcpy(s_text[row], line, sizeof(line));
        s_style[row] = (uint8_t)style;
        s_valid[row] = 1;
        return;
    }

    // 3. 样式没变：只重画内容不同的字符（字宽 ASCII_Font20.Width = 10 像素）
    for (col = 0; col < UI_COLS; col++) {
        if (line[col] != s_text[row][col]) {
            NEUQ_LCD200_DisplayChar((uint16_t)(col * ASCII_Font20.Width), y, (uint8_t)line[col]);
            s_text[row][col] = line[col];
        }
    }
}
