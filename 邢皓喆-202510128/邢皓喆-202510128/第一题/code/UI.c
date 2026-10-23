#include "NEUQ_lib_include.h"

volatile uint8_t* next = &TASK_CMD;
volatile uint8_t current = 1;

void NEUQ_MANU_Display(void)
{
    static uint8_t initialized = 0;
    static uint8_t displayed_current = 0;
    static uint8_t displayed_next = 0;
    uint8_t current_mode = current;
    uint8_t next_mode = *next;
    char line[32];

    /* Clear the screen and draw fixed text only on the first call. */
    if (!initialized)
    {
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_DisplayString(8, 8, "LED CONTROL");
        initialized = 1;
    }

    /* The LCD font draws its background too, so no clear is needed. */
   // if (!initialized || current_mode != displayed_current)
  //  {
        snprintf(line, sizeof(line), "Current Mode : %u", (unsigned int)current_mode);
        NEUQ_LCD200_DisplayString(8, 80, line);
        displayed_current = current_mode;
   // }

//if (!initialized || next_mode != displayed_next)
  //  {
        snprintf(line, sizeof(line), "Next Mode    : %u", (unsigned int)next_mode);
        NEUQ_LCD200_DisplayString(8, 112, line);
        displayed_next = next_mode;
 //   }

}

void UI_init(void)
{
    /* A completed LED cycle alone does not mean the displayed mode changed. */
    NEUQ_MANU_Display();
}
