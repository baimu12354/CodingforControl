#ifndef _MENU_H_
#define _MENU_H_

#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Main.h"

enum {
    MENU_MAIN = 0, MENU_RAW, MENU_FILTER, MENU_ATTITUDE,
    MENU_RAW_AX, MENU_RAW_AY, MENU_RAW_AZ, MENU_RAW_GX, MENU_RAW_GY, MENU_RAW_GZ,MENU_ALPHA_ACC, MENU_ALPHA_GYRO,
     MENU_VOFA_ENABLE,MENU_AX, MENU_AY, MENU_AZ, MENU_GX, MENU_GY, MENU_GZ,
    MENU_ROLL, MENU_PITCH, MENU_YAW
};
typedef struct {
     char *name;
    int parent_id;
    float *value;
    uint8_t editable;
} MenuInfo;

typedef struct MenuNode {
     MenuInfo *info;
    struct MenuNode *parent;
    struct MenuNode *child;
    struct MenuNode *prev;
    struct MenuNode *next;
} MenuNode;



extern float menu_value[17];
 extern MenuNode nodes[22];
 extern MenuNode *page;
 extern MenuNode *selected;
 extern uint8_t editing;
 extern float edit_value;
 extern MenuNode *lastselected;
 
 extern MenuInfo menu_table[] ;


uint8_t MenuNodeId(const MenuNode *node);

void Menu_Init(void);


void MenuRefresh(void);



#endif

