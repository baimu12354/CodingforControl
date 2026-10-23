#ifndef KEY_H
#define KEY_H
#include "NEUQ_lib_include.h"
#define KEY_UP PE7
#define KEY_DOWN PE9
#define KEY_FORWARD PE10
#define KEY_BACK PE8
#define KEY_COUNT 4
#define LEY_SCAN_PERIOD 10
typedef enum{
 KEY_NONE = 0,
 KEY_UP_P,
 KEY_DOWN_P,
 KEY_FORWAED_P,
 KEY_BACK_P,

}KEY_EVENT_ENUM;
typedef enum{
    KEY_ACT_NONE = 0,
    KEY_PRESS,
}KEY_ACT_ENUM;
void KEY_Init(void);
void KEY_Scan(void);
KEY_EVENT_ENUM KEYGet(void);

#endif /* KEY_H */
