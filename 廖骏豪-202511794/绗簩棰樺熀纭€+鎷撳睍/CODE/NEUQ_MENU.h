/*
 * NEUQ_MENU.h
 * 两级菜单：一级选功能，二级是三个页面
 *
 * 一级 MAIN MENU
 *   1. AXIS DATA        六轴数据显示
 *   2. WAVE & FILTER    波形与滤波设置
 *   3. ATTITUDE         姿态解算
 *
 * 按键分配，一个键只干一件事，不做短按长按两个功能
 *   UP / DOWN   移动光标；进入编辑后是加一减一，按住0.35秒开始连发
 *   FORWARD     进入选中的页面；设置页里是开始编辑/确认；姿态页里是重新校准
 *   BACK        返回上一级；编辑中的话就是放弃这次修改
 *
 * OK和BACK故意不做长按。如果做成"短按进入编辑、长按直接保存"，
 * 手重一点按久了就会一路存下去，想改参数反而进不去编辑，
 * 现象就是"设置页里怎么按都编辑不了"。现在这两个键按住多久都只在松手时
 * 产生一次SHORT事件，行为是确定的。长按连发只留给UP/DOWN调参数。
 *
 * 设置页底部会显示当前处在哪个模式，免得搞混：
 *   浏览：OK:EDIT  BACK:RET
 *   编辑：EDIT: UP/DN ADJ  OK:SET   这一行是蓝的，而且当前行行首会变成 *
 */
#ifndef __NEUQ_MENU_H
#define __NEUQ_MENU_H

#include "NEUQ_lib_include.h"
#include "NEUQ_Key.h"

typedef enum{
    MENU_PAGE_MAIN = 0,         // 一级功能选择页
    MENU_PAGE_DATA,             // 六轴数据显示
    MENU_PAGE_WAVE,             // 波形与滤波设置
    MENU_PAGE_ATT,              // 姿态解算
    MENU_PAGE_NUM
} MENU_PAGE_ENUM;

// 菜单复位到一级页面并整屏重画
void NEUQ_Menu_Init(void);

// 处理一个按键事件，由NEUQ_App.c从按键驱动取出来后转交
void NEUQ_Menu_HandleKey(KEY_EVENT_ENUM key_event);

// 周期刷新当前页面的数值，100ms调一次
// 只重画会变的数值，静态文字进入页面时画一次，所以刷屏开销不大
void NEUQ_Menu_Update(void);

MENU_PAGE_ENUM NEUQ_Menu_GetPage(void);

// 在设置页右下角显示一条状态，比如SAVED / UNSAVED / SAVE FAIL
void NEUQ_Menu_SetStatus(const char *text);

// 往调试串口USART1(PA9/115200)打一行字，自带超时
// 主循环里这种会被反复调用的地方都用它，不要用printf。
// 库里的printf底层是没有超时的while死等，串口一出问题整个系统就卡住了。
void NEUQ_Menu_DebugPrint(const char *text);

// 上电自检用的简单提示画面，清屏加两行字
// 例如 NEUQ_Menu_BootScreen("MPU6050 OK", "GYRO CALIBRATING");
void NEUQ_Menu_BootScreen(const char *line1, const char *line2);

#endif /* __NEUQ_MENU_H */
