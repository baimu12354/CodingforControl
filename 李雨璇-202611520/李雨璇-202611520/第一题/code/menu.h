#ifndef MENU_H_
#define MENU_H_

#include "NEUQ_lib_include.h"

void Menu_Init(void);
void Menu_Tick(void);    //LED闪烁时钟中断（每1ms调用）
void Menu_Task(void);    //处理按键事件 + 刷新页面

#endif