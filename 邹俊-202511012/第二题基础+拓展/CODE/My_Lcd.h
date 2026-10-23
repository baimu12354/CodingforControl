/*********************************************************************************************************************
* 文件名称          My_Lcd
* 功能说明          基于库 NEUQ_LCD200 的"按行显示文字"：横屏 320x240，10 行 x 32 列（ASCII 10x20 字体）
*
*                   为什么不直接调用 NEUQ_LCD200_DisplayString：
*                   SPI 屏每画一个字符都要传 10x20 个像素，整页重画要几十毫秒，还会闪屏。
*                   这里给每一行记住"屏幕上现在显示的是什么"，再次显示同一行时只重画变了的那几个字符，
*                   刷新数值时通常只传几个字符，又快又不闪；切换页面也不用整屏清屏。
*
*                   使用方法：
*                   My_Lcd_init();
*                   My_Lcd_print(3, UI_STYLE_NORMAL, "Batt: %5.2f V", voltage);   // 用法和 printf 一样
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#ifndef _MY_LCD_H_
#define _MY_LCD_H_

#include <stdint.h>

#define UI_ROWS             (10)        // 行数
#define UI_COLS             (32)        // 每行字符数（320 像素 / 字宽 10）
#define UI_ROW_PITCH        (24)        // 行距，单位像素（字高 20 + 行间隔 4）

typedef enum {                          // 行的显示样式（字色 / 底色）
    UI_STYLE_NORMAL = 0,                // 黑字白底：普通内容
    UI_STYLE_TITLE,                     // 白字深蓝底：页面标题
    UI_STYLE_SELECT,                    // 白字蓝底：光标所在行
    UI_STYLE_EDIT,                      // 黑字黄底：正在编辑的参数
    UI_STYLE_GOOD,                      // 深绿字：正常 / 运行中
    UI_STYLE_WARN,                      // 红字：警告 / 请等待
    UI_STYLE_HINT,                      // 灰字：操作提示
    UI_STYLE_NUM
} UI_Style_Enum;

void My_Lcd_init(void);                                                    // 初始化屏幕（白底）
void My_Lcd_print(uint8_t row, UI_Style_Enum style, const char *fmt, ...); // 显示一整行：不足补空格，超出截断
void My_Lcd_invalidate(void);                                              // 忘掉缓存，下次显示时每行整行重画

#endif
