#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_
#include "NEUQ_lib_include.h"

void NEUQ_Menu_Init(void);   // 初始化 LCD 并显示首屏
void NEUQ_Menu_Task(void);   // 主循环调用：按键处理 + 电压采样 + 显示刷新
void GYR_Menu_Show(void);    // 六轴数据显示（中断/任务里调用需注意耗时）
#endif
