
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
volatile uint8_t led_stage = 1;
volatile uint16_t led_elapsed_ms = 0;
volatile uint8_t led_stage_changed = 1;


uint8_t led_level(uint8_t stage, uint16_t t)
{
    switch (stage) {
        case 1: return (t < 1000) ;
        case 2: return (t < 200) ;
        case 3: return ((t < 200) || (t >= 400 && t < 600)) ;
        case 4: return ((t < 200) || (t >= 400 && t < 1000)) ;
        default: return 0;
    }
}
