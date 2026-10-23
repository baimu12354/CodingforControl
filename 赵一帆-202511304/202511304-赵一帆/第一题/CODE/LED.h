#ifndef _LED_H_
#define _LED_H_
#include "NEUQ_lib_include.h"

#define LED_PIN       PA8
#define KEY_UP_PIN    PE8
#define KEY_DOWN_PIN  PE9
#define KEY_ENTER_PIN PE7
#define KEY_BACK_PIN  PE10

typedef enum { KEY_NONE=0, KEY_UP, KEY_DOWN, KEY_ENTER, KEY_BACK } KeyEvent_t;

typedef struct {
    uint8_t  current_mode;   // 当前方案
    uint8_t  target_mode;    // 待执行方案
    uint32_t tick;           // 1ms 计数
} LedSystem_t;

extern LedSystem_t ledSys;

void LED_Init(void);
void LED_Task(void);
void Key_Scan(void);       
void LED_Menu_Task(void);   // 上/下切换方案
void LED_LCD_Task(void);

#endif