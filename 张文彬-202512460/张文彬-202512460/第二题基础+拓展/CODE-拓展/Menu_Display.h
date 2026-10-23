#ifndef MENU_DISPLAY_H
#define MENU_DISPLAY_H

/* 二级菜单、页面切换、参数编辑及 LCD 显示。 */
#include <stdint.h>
#include "Imu_Config.h"
void Menu_Display_EnterPage(AppPage page);
void Menu_Display_ProcessKeys(uint8_t keys);
void Menu_Display_Refresh(void);
uint8_t Menu_Display_StreamEnabled(void);

#endif /* MENU_DISPLAY_H */
