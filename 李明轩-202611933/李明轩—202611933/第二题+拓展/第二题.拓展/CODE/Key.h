#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>

// 页面编号
#define PAGE_MENU      0
#define PAGE_DISPLAY   1   // 六轴数据显示
#define PAGE_FILTER    2   // 波形与滤波设置
#define PAGE_ATTITUDE  3   // 姿态解算

#define MENU_NUM 3

// 当前页面（Task_Gyro.c 判断是否采样/上传时读取）
extern uint8_t page;

// 界面初始化：按键引脚 + 屏幕
void Key_Init(void);

// 界面主循环：读按键 + 菜单逻辑 + 页面绘制 + 动态刷新
void Key_Loop(void);

#endif
