#ifndef _NEUQ_MENU_H_
#define _NEUQ_MENU_H_


#define LED_PIN         PA8
/* 按键引脚 KEY_UP_PIN(PE9) / KEY_DOWN_PIN(PE8) 已统一移入 NEUQ_KEY.h 管理 */
/* PA3 电压显示：ADC1_CH3，12 位，菜单第三行实时刷新（验收项） */

/* ============================ 接口声明 ============================ */
void LED_Menu_Init(void);   /* 初始化：GPIO/LCD/ADC/定时器 + 画首屏 */
void LED_Menu_Loop(void);   /* 主循环：扫描按键改待执行方案 + 周期刷新电压 + 刷新菜单 */
void LED_Tick(void);        /* 1ms 中断：驱动 LED 状态机（段/方案切换） */

#endif 