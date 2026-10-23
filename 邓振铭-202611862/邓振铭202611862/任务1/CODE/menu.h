#ifndef _MENU_H_
#define _MENU_H_

#include"stdint.h"
extern float V;

void menu_show(void);
void menu_init(void);
void menu_task(void);
uint8_t menu_update(KEY_EVENT_ENUM*key_out);
#endif