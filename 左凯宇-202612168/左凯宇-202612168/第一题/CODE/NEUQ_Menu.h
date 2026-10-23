#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

//此处无需过多函数，正常库即可
#include <stdio.h>

typedef struct{
    int32_t action;
    int32_t pre;
}NEUQ_Menu_Params;

extern NEUQ_Menu_Params params;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);

#endif