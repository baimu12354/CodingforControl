#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include<stdio.h>


void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);
int8_t NEUQ_Menu_GetPage(void);
int8_t NEUQ_Menu_GetActiveMode(void);
void NEUQ_Menu_SetActiveMode(int8_t mode);



#endif