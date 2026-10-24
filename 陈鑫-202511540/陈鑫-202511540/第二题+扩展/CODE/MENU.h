#ifndef MENU_H
#define MENU_H


#include "NEUQ_lib_include.h"
#include "NEUQ_KEY.h"

typedef enum{
    PAGE_MAIN=0,
    PAGE_SIX_AXIS,
    PAGE_WAVE_FILTER,
    PAGE_ATTITUDE,
}MENUPAGE;

typedef enum{
    CUR_SEND=0,
    CUR_AXIS,
    CUR_ALPHA,
    CUR_GYRO,
    CUR_ACC,
    CUR_SAVE,
}WAVECUR;

typedef enum{
    EDIT_OFF=0,
    EDIT_ON,
}EDITSTATE;

extern MENUPAGE g_menu_page;
extern uint8_t g_menu_cursor;
extern EDITSTATE g_edit_state;

void menu_init(void);

void menu_key_handle(KEY_EVENT_ENUM key_event);

void menu_show(void);




#endif