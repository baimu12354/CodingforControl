#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include"stdint.h"
#include"NEUQ_KEY.h"
typedef enum{
    MENU_NONE=-1,
    MENU_LED_TEXT,
    MENU_STRATEGY1,
    MENU_STRATEGY2,
    MENU_STRATEGY3,
    MENU_STRATEGY4,
    MENU_COUNT
}MENU_ID;//菜单编号


typedef struct
{
   const char *name;//当前页面的名字
   MENU_ID parent_id;//父节点地址
   int32_t *value;//当前页面参数地址
   int32_t max;
   int32_t min;
   int32_t step;
}Menuconfig;//节点内容配置

//链表节点
typedef struct MENU_NODE{
const Menuconfig *info; //节点内容
struct MENU_NODE *parent;
struct MENU_NODE *child;
struct MENU_NODE *prev;
struct MENU_NODE *next;
}MENU_NODE;

extern MENU_NODE nodes[MENU_COUNT];//存储所有节点，nodes[i]就是一个节点
extern MENU_NODE *page;//当前页面的节点
extern MENU_NODE *select;//选择高亮项的节点
extern uint8_t editing;//0->浏览模式，1->编辑模式
extern uint32_t edit_temp;//用于临时存储编辑模式下更改变量的值



void Menu_Init(void);
uint8_t Menu_Update(KEY_EVENT_ENUM Key);
void Menu_Show(void);
void Menu_Task(void);

#endif