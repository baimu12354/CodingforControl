#ifndef _NEUQ_MENU_H_
#define _NEUQ_MENU_H_

#include<stdio.h>
#include<stdint.h>

typedef struct{
    int32_t FilterParams;   // 滤波参数 1~100，对应 alpha = FilterParams/100.0f
    int32_t UploadAxis;     // 发送轴：0=关 1=Ax 2=Ay 3=Az 4=Gx 5=Gy 6=Gz
    int32_t UploadOn;       // 上传开关：0=停止 1=发送
} NEUQ_Menu_Params;

extern NEUQ_Menu_Params params;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);


#endif
