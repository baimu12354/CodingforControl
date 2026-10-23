
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
uint8_t current_insert = 0;
extern volatile uint8_t led_stage;
void lcd_draw(uint8_t current_insert)
{
    char line[32];

    NEUQ_LCD200_ClearRect(0, 0, 240, 320);

    // NEUQ_LCD200_DisplayString(8, 38, line);
    sprintf(line, "Now: S%d  Select: S%d", (int)led_stage,(int)(current_insert + 1));
    // NEUQ_LCD200_DisplayString(8, 0, line);
    // // sprintf(line, "PA8: %s", led_level(led_stage, led_elapsed_ms) ? "ON" : "OFF");
    // NEUQ_LCD200_DisplayString(8, 60-38, line);
    // sprintf(line, "PA3: %.2f V", voltage);
    // NEUQ_LCD200_DisplayString(8, 128-38*2, line);
    // sprintf(line, "Next: %s", auto_cycle ? "cycle" : "selected stage");
    NEUQ_LCD200_DisplayString(8, 158-38*2, line);
    sprintf(line, "%c Stage 1", current_insert == 0 ? '>' : ' '); NEUQ_LCD200_DisplayString(8, 184-38*2, line);
    sprintf(line, "%c Stage 2", current_insert == 1 ? '>' : ' '); NEUQ_LCD200_DisplayString(8, 204-38*2+10, line);
    sprintf(line, "%c Stage 3", current_insert == 2 ? '>' : ' '); NEUQ_LCD200_DisplayString(8, 224-38*2+20, line);
    sprintf(line, "%c Stage 4", current_insert == 3 ? '>' : ' '); NEUQ_LCD200_DisplayString(8, 244-38*2+30, line);
    // NEUQ_LCD200_DisplayString(8, 274, "ENTER: manual  BACK: auto");

}