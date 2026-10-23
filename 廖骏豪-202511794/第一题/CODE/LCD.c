#include "NEUQ_lib_include.h"
#include "LCD.h"

/*  单页菜单，横屏 320x240，字库 ASCII_Font24（12x24 一个字符）
 *
 *      LED TIMING 2s  PA8
 *      CUR  SCHEME : 2        当前正在跑的方案
 *      NEXT SCHEME : 3        按键选的，下个周期才生效
 *      BAT VOLTAGE : N/A
 *      LED  STATE  : ON      右边方块跟着亮灭变绿/灰
 *      KEY UP/DN: SELECT
 *
 *  哪一项变了才重画哪一项，不整屏刷，省时间也不闪
 */

#define LINE_CUR     44
#define LINE_NEXT    80
#define LINE_VOLT    116
#define LINE_STATE   152
#define LINE_HINT    188

#define VAL_X        168       // 数值显示起点
#define VAL_W        96        // 重画宽度，右边留到 263，272 开始放指示灯
#define BOX_X        272
#define BOX_W        32
#define BOX_H        24

// 记一下屏上已经显示的是什么，一样就不重画了
static uint8_t  shown_cur  = 0xFF;
static uint8_t  shown_pend = 0xFF;
static uint8_t  shown_on   = 0xFF;
static uint32_t shown_mv   = LCD_VOLTAGE_INVALID - 1;
static uint32_t refresh_tick = 0;

// 先把这一块擦掉再写，不然新旧字符会叠在一起
static void show_text(uint16_t x, uint16_t y, uint16_t w, char *s, uint32_t color)
{
    NEUQ_LCD200_ClearRect(x, y, w, 24);
    NEUQ_LCD200_SetPenColor(color);
    NEUQ_LCD200_DisplayString(x, y, s);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
}

// LED 状态方块，亮绿灭灰，外面描一圈黑边
static void draw_led_box(uint8_t on)
{
    uint16_t x2 = BOX_X + BOX_W - 1;
    uint16_t y2 = LINE_STATE + BOX_H - 1;

    NEUQ_LCD200_SetBackColor(on ? LCD_GREEN : LIGHT_GREY);
    NEUQ_LCD200_ClearRect(BOX_X, LINE_STATE, BOX_W, BOX_H);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);

    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_DrawLine(BOX_X, LINE_STATE, x2, LINE_STATE);
    NEUQ_LCD200_DrawLine(BOX_X, y2,         x2, y2);
    NEUQ_LCD200_DrawLine(BOX_X, LINE_STATE, BOX_X, y2);
    NEUQ_LCD200_DrawLine(x2,    LINE_STATE, x2,    y2);
}

void LCD_Menu_Init(void)
{
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font24);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_Clear();

    NEUQ_LCD200_DisplayString(8, 8, "LED TIMING 2s  PA8");

    NEUQ_LCD200_DisplayString(0, LINE_CUR,   "CUR  SCHEME :");
    NEUQ_LCD200_DisplayString(0, LINE_NEXT,  "NEXT SCHEME :");
    NEUQ_LCD200_DisplayString(0, LINE_VOLT,  "BAT VOLTAGE :");
    NEUQ_LCD200_DisplayString(0, LINE_STATE, "LED  STATE  :");

    NEUQ_LCD200_SetPenColor(LCD_GREY);
    NEUQ_LCD200_DisplayString(0, LINE_HINT, "KEY UP/DN: SELECT");
    NEUQ_LCD200_SetPenColor(LCD_BLACK);

    // 后面第一次刷新的时候把动态内容补上去
    shown_cur  = 0xFF;
    shown_pend = 0xFF;
    shown_on   = 0xFF;
    shown_mv   = LCD_VOLTAGE_INVALID - 1;
    refresh_tick = HAL_GetTick();
}

void LCD_Menu_Update(uint8_t cur, uint8_t pend, uint8_t on, uint32_t bat_mv)
{
    char     buf[24];
    uint32_t now = HAL_GetTick();

    if(now - refresh_tick < 20) return;      // 20ms 刷一次够了
    refresh_tick = now;

    if(cur != shown_cur)
    {
        shown_cur = cur;
        sprintf(buf, "%d", cur);
        show_text(VAL_X, LINE_CUR, VAL_W, buf, LCD_BLACK);
    }

    if(pend != shown_pend)
    {
        shown_pend = pend;
        sprintf(buf, "%d", pend);
        show_text(VAL_X, LINE_NEXT, VAL_W, buf, LCD_BLUE);
    }

    if(on != shown_on)
    {
        shown_on = on;
        show_text(VAL_X, LINE_STATE, VAL_W, on ? "ON" : "OFF",
                  on ? DARK_GREEN : LCD_GREY);
        draw_led_box(on);
    }

    if(bat_mv != shown_mv)
    {
        shown_mv = bat_mv;

        if(bat_mv == LCD_VOLTAGE_INVALID)
        {
            show_text(VAL_X, LINE_VOLT, VAL_W, "N/A", LCD_RED);
        }
        else
        {
            sprintf(buf, "%u.%02u V", bat_mv / 1000, (bat_mv % 1000) / 10);
            show_text(VAL_X, LINE_VOLT, VAL_W, buf, LCD_BLACK);
        }
    }
}
