/*********************************************************************************************************************
* 文件名称   menu
* 功能说明   编码式菜单：光标、进入、返回、参数编辑，供主循环调用
* 使用方法   开机 Menu_Init()；主循环 Key_Scan()->Menu_OnEvent()->Menu_Render()
*********************************************************************************************************************/
#ifndef _MENU_H_
#define _MENU_H_

#include "NEUQ_lib_include.h"
#include "key.h"

// 进入一级菜单（开机调用一次）
void Menu_Init(void);
// 把一个按键事件交给菜单引擎处理（光标/进入/返回/编辑/确认）
void Menu_OnEvent(KeyEvent_t ev);
// 把当前页面画到屏幕
void Menu_Render(void);
// 只有“显示页(六轴/姿态)”按50ms周期刷新；列表页不刷
void Menu_RefreshDisplay(void);

#endif
