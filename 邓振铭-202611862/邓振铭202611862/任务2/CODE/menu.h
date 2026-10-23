#ifndef _MENU_H_
#define _MENU_H_
typedef enum{

menu_none=0,
menu_main,
menu_datashow,
menu_lvbo_set,
menu_jitaijiesuan,
menu_count,
}menuid;
#include"NEUQ_lib_include.h"
typedef enum{
    page_main=0,
    page_datashow,
    page_lvbo_set,
    page_jitaijiesuan,
}MenuPage_enum;
typedef struct 
{
menuid page_id;
const char*page_name;
void (*page_draw)(void);

    /* data */
}menutable_t;
extern const menutable_t menutable[];
static void draw_main(void);
static void draw_data(void);
static void draw_filter(void);
static void draw_attitude(void);
extern MenuPage_enum MenuPage_now;
extern uint16_t selected;
void menu_init(void);
void menu_buildlink(void);
void menu_show(void);
uint16_t menu_update(void);
void menu_task(void);

#endif