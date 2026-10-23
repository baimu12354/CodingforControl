#ifndef _IMUMENU_H_
#define _IMUMENU_H_
#include "NEUQ_lib_include.h"

typedef enum { KEY_NONE=0, KEY_UP, KEY_DOWN, KEY_ENTER, KEY_BACK } KeyEvent_t;
typedef enum { PAGE_MAIN=0, PAGE_IMU, PAGE_FILTER, PAGE_ATTITUDE } Page_t;

#define KEY_UP_PIN    PE8
#define KEY_DOWN_PIN  PE9
#define KEY_ENTER_PIN PE7
#define KEY_BACK_PIN  PE10

void Key_Init(void);
void Key_Scan(void);
void Menu_Init(void);
void Menu_Task(void);
void Menu_LCD_Task(void);

#endif