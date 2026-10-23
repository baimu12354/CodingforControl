#ifndef MENU_H
#define MENU_H

#include "app_common.h"

void Menu_Init(void);
void Page_Enter(Page_Id p);
void Refresh_Values(void);
void Handle_Keys(uint8_t up, uint8_t down, uint8_t enter, uint8_t back);
uint8_t Key_Read(Key_Id id);

#endif
