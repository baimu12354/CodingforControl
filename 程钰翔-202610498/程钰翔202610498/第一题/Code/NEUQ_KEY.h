#ifndef _NEUQ_KEY_H_
#define _NEUQ_KEY_H_

#include "NEUQ_lib_include.h" 

typedef void(*funt_event)(void);

typedef struct {
    GPIO_Ports_ENUM  Port;
   // GPIO_Level Last_state;
   //GPIO_Level Now_state;
    funt_event Press_event;
    
}Key_type;

void NEUQ_Key_Init(Key_type *key,GPIO_Ports_ENUM pin,funt_event Press_event);
uint8_t Key_Scan(Key_type *key);

#endif