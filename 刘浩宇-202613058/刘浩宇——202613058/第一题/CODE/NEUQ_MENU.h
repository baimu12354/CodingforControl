#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

extern volatile uint8_t current_scheme;
extern volatile uint8_t next_scheme;

void NEUQ_LCD_INIT(void);//屏幕初始化
void LED_Handler(void);  //LED控制
void NEUQ_MENU_INIT(void);//菜单初始化（链表建立，屏幕初始化,LED初始化）
void NEUQ_MENU_TASK(void);

#endif
