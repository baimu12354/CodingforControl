#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_
#include <stdio.h>

typedef struct{
    int32_t ;
    int32_t ;
    int32_t ;
    int32_t ;
}NEUQ_Menu_Params;

extern NEUQ_Menu_Params params;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);

#endif 