#ifndef _NEUQ_MENU_H
#define _NEUQ_MENU_H
#include <stdio.h>
#include <stdint.h>
#include "key.h"
#ifdef NEUQ_MENU_SIM
#else
#include "NEUQ_lib_include.h"
#endif
#ifndef NEUQ_MENU_SIM

#define LED_PORT        (TIM1_CH1_PA8)
#define LED_PWM_FREQ    (1000u)       
typedef struct 
{
    int8_t now;
    int8_t next;
}menuparamas;
typedef enum{
    first,
    second,
    third,
    fourth,
    menucount,
}menuid;
typedef struct
{
    const char      *zhushi;
    menuid          next;
    menuid          up;
    int8_t         *xianzai;
    int8_t          id;
}menuconfig;

typedef struct menunode
{
    const  menuconfig     *info;
    struct menunode       *prev;
    struct menunode       *next;
} menunode;


extern menuparamas paramas;                     
void Menu_BuildLinks(void);
uint8_t menutask(void);

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);
int8_t NEUQ_Menu_Update(KEY_EVENT_ENUM key);
void TURN(int8_t);                                
#endif  /* NEUQ_MENU_SIM */
#endif  

