#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include <stdio.h>

extern volatile uint8_t current;
extern volatile uint8_t next;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);
void NEUQ_Menu_Refresh(void);
void Menu_ShowStatus(void);

#endif
