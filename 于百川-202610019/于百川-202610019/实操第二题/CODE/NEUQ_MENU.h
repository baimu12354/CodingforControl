#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include<stdio.h>

typedef struct{
    int32_t send_axis;      /* 发送轴 0..5 */
    int32_t send_enable;    /* 波形发送开关 0/1 */
    int32_t alpha;          /* 滤波系数 ×100 (5..95) */
} NEUQ_Menu_Params;

extern volatile NEUQ_Menu_Params Params;   /* 中断(10ms任务)读 / 主循环写，volatile 防优化缓存 */

void NEUQ_Menu_Init(void);   
void NEUQ_Menu_Task(void);

#endif