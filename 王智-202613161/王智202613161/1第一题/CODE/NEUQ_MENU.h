#ifndef _NEUQ_MENU_H_
#define _NEUQ_MENU_H_

#include<stdio.h>
#include<stdint.h>

  // 菜单名称
typedef enum {
    PlanNone =-1,
    Plan1 =0,
    Plan2,
    Plan3,
    Plan4,
    PlanCount, //方案个数
}PLANS_ID;

  //菜单通用配置
typedef struct{
    const char *name;
    int8_t planid; 
}MenuConfig;

  //节点通用结构
typedef struct NODE{
    const  MenuConfig *info;  //内容
    struct NODE *prev;       //前项
    struct NODE *next;       //后项
} NODE;

extern int8_t Pending;
extern int8_t Working;
extern NODE *working;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Mask(void);
PLANS_ID NEUQ_Menu_WorkGet(void);

#endif
