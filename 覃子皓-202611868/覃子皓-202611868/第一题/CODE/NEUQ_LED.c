#include "NEUQ_lib_include.h"


void NEUQ_LED_init(void){       
    NEUQ_GPIO_init(NEUQ_LED_PIN, GPIO_Mode_Output_PushPull, GPIO_Low);  //推挽，低电平
}
void NEUQ_LED_on(void){
    NEUQ_GPIO_write(NEUQ_LED_PIN, GPIO_High);
}
void NEUQ_LED_off(void){
    NEUQ_GPIO_write(NEUQ_LED_PIN, GPIO_Low);
}
