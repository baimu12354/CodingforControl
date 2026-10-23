#include "NEUQ_lib_include.h"

volatile uint32_t cycle_timer = 0;
volatile KEY_EVENT_ENUM current_mode = EVENT1;   
volatile KEY_EVENT_ENUM pending_mode = EVENT1;
// 上电默认方案一

void LED_TASK(void){
    uint8_t led_state = 0;
    
    KEY_EVENT_ENUM key = NEUQ_Key_EventGet();
    if (key != EVENT_NONE){
        pending_mode = key;          
    }

    
    if (cycle_timer >= LED_CYCLE)
    {
        cycle_timer = 0;
        current_mode = pending_mode;
    }
    
    switch (current_mode)
    {
    case EVENT1:
        if (cycle_timer < 1000)
            led_state = 1;
        else led_state = 0;
        break;
    
    case EVENT2:
        if (cycle_timer < 200)
            led_state = 1;
        else led_state = 0;
        break;
    
    case EVENT3:
        if (cycle_timer < 200)
            led_state = 1;
        else if(cycle_timer < 400)
            led_state = 0;
        else if(cycle_timer < 600)
            led_state = 1;
        else led_state = 0;
        break;
        
    case EVENT4:
        if (cycle_timer < 200)
            led_state = 1;
        else if(cycle_timer < 400)
            led_state = 0;
        else if(cycle_timer < 1000)
            led_state = 1;
        else led_state = 0;
        break;
    
    }

    
    NEUQ_GPIO_write(LED_PIN,(GPIO_Level)led_state);
    
    
}