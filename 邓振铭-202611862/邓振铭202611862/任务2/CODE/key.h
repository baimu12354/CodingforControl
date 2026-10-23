#ifndef _KEY_H_
#define _KEY_H_
#include"NEUQ_lib_include.h"
#define UP  PE10
#define DOWN PE8
#define FORWARD PE7
#define BACK PE9
#define RIGHT PE11
typedef enum{
UP_1=0,
DOWN_1,
FORWARD_1,
BACK_1,
RIGHT_1
}KEY_EVENT_ENUM;
typedef enum{
KEY_ACT_NONE=0,
KEY_ACT_HAVE

}KEY_ACT_ENUM;
void key_init(void);
void key_scan(void);
uint16_t key_eventget(void);

#endif