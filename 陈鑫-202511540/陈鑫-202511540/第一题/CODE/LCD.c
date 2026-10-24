#include "NEUQ_lib_include.h"

void LCD_init(void)
{
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_RED);
    NEUQ_LCD200_DisplayString(10,0,"LED Timing");
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_DisplayString(10,40,"Cur:");
    NEUQ_LCD200_DisplayString(10,80,"Next:");
}

void Lcd_Show(void)
{
    static uint8_t last_cur=0xFF;
    static uint8_t last_pend=0xFF;
    if(last_cur!=g_cur_scheme||last_pend!=g_pending_scheme){
    NEUQ_LCD200_SetPenColor(LCD_BLUE);
    NEUQ_LCD200_ClearRect(70,40,12,24);NEUQ_LCD200_Display_int32(70,40,g_cur_scheme+1,1);
    NEUQ_LCD200_ClearRect(70,80,12,24);NEUQ_LCD200_Display_int32(70,80,g_pending_scheme+1,1);
    last_cur=g_cur_scheme;
    last_pend=g_pending_scheme;
    }
}