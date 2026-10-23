#ifndef _MENU_H_
#define _MENU_H_
#include "NEUQ_lib_include.h"
typedef struct
{
int16_t axis;
int16_t start;
float yaw;
float pitch;
float roll;
float gyro_bias;

}Menu_Params;

extern Menu_Params param;
void MENU_Init(void);
void MENU_Task(void);
void MENU_Flash(void);





#endif 