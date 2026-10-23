#include "NEUQ_lib_include.h"

static uint8_t menu_current_last = 0;
static uint8_t menu_pending_last = 0;

static void NEUQ_Menu_DrawPlanLine(uint16_t y, const char *name, uint8_t plan)
{
    char line[24];

    snprintf(line, sizeof(line), "%s:Plan%d", name, plan);
    NEUQ_LCD200_DisplayString(8, y, line);
}

void NEUQ_Menu_Init(void)
{
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(8, 8, "===LED MENU===");

    menu_current_last = NEUQ_Led_GetCurrentPlan();
    menu_pending_last = NEUQ_Led_GetPendingPlan();
    NEUQ_Menu_DrawPlanLine(50, "Current", menu_current_last);
    NEUQ_Menu_DrawPlanLine(80, "Pending", menu_pending_last);
}

void NEUQ_Menu_Task(void)
{
    uint8_t current = NEUQ_Led_GetCurrentPlan();
    uint8_t pending = NEUQ_Led_GetPendingPlan();

    if (current != menu_current_last)
    {
        NEUQ_Menu_DrawPlanLine(50, "Current", current);
        menu_current_last = current;
    }

    if (pending != menu_pending_last)
    {
        NEUQ_Menu_DrawPlanLine(80, "Pending", pending);
        menu_pending_last = pending;
    }
}
