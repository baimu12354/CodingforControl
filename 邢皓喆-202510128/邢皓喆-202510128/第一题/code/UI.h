#ifndef _UI_H
#define _UI_H
extern volatile uint8_t current;
extern volatile uint8_t UI_REFRESH;
#include "NEUQ_lib_include.h"
void NEUQ_MANU_Display(void);
void UI_init(void);
#endif