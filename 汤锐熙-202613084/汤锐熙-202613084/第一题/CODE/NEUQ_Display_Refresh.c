#include "NEUQ_Display_Refresh.h"

/* 共享状态定义（初值：当前方案 = 1，待执行方案 = 1） */
volatile uint8_t g_led_current = 1;
volatile uint8_t g_led_next    = 1;
volatile uint8_t g_led_dirty   = 1;

/* 屏幕布局：横屏 320x240；默认 ASCII_Font24 为 24 宽 x 12 高 */
#define LABEL_X   20
#define NUM_X     160
#define ROW1_Y    40
#define ROW2_Y    90

void NEUQ_Display_Refresh_Init(void){
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_Clear();

    // 库内 LCD 只有 ASCII 字库，用英文标签：
    //   CUR  = 当前方案
    //   NEXT = 待执行方案
    NEUQ_LCD200_DisplayString(LABEL_X, ROW1_Y, "CUR:");
    NEUQ_LCD200_DisplayString(LABEL_X, ROW2_Y, "NEXT:");

    NEUQ_Display_Refresh_Update();
}

void NEUQ_Display_Refresh_Update(void){
    // 方案号是 1~4 的个位数，固定单字符位显示，每次写都会连同背景覆盖旧数字
    NEUQ_LCD200_Display_int32(NUM_X, ROW1_Y, g_led_current, 1);
    NEUQ_LCD200_Display_int32(NUM_X, ROW2_Y, g_led_next, 1);
}
