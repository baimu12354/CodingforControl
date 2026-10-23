#ifndef _NEUQ_MENU_H_
#define _NEUQ_MENU_H_

#include <stdint.h>
#include "NEUQ_Key.h"

typedef enum {
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_6AXIS,
    MENU_FILTER,
    MENU_ATTITUDE,
    MENU_FILTER_KP,
    MENU_FILTER_AXIS,
    MENU_FILTER_SEND,
    MENU_COUNT
} MenuID;

typedef struct {
    const char *name;
    MenuID parent_id;
    int32_t *value;
} MenuConfig;

typedef struct MenuNode {
    const MenuConfig *info;
    struct MenuNode *parent;
    struct MenuNode *child;
    struct MenuNode *prev;
    struct MenuNode *next;
} MenuNode;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);
void NEUQ_Menu_SendKey(KeyEventEnum key);
MenuNode *NEUQ_Menu_GetCurrentNode(void);
uint8_t NEUQ_Menu_GetCurrentPageID(void);

#endif
