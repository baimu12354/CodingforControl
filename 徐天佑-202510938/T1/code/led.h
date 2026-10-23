#ifndef __LED_H__
#define __LED_H__
extern volatile uint8_t led_stage ;
extern volatile uint16_t led_elapsed_ms ;
extern volatile uint8_t led_stage_changed ;
uint8_t led_level(uint8_t stage, uint16_t t);

#endif