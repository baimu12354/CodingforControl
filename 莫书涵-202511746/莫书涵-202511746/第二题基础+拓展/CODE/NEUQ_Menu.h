#ifndef NEUQ_MENU_H_
#define NEUQ_MENU_H_

#include <stdio.h>

#define Dynamic_refresh_time      100    //动态参数刷新时间 ms

#define ALPHA1000_MIN        50
#define ALPHA1000_MAX        990
#define ALPHA1000_MOREN      300
#define ALPHA_SAVE_SECTOR    NEUQ_FLASH_LAST_SECTOR_ID
#define ALPHA_SAVE_ADDRESS   (ALPHA_SAVE_SECTOR * NEUQ_FLASH_SECTOR_SIZE)

//存放可修改参数
typedef struct{
    int32_t alpha_1000[6];  //滤波系数*1000
    int8_t send_axis;       //发送哪轴波形 0=Xa 1=Ya 2=Za 3=Xg 4=Yg 5=Zg
    int8_t send_require;    //发送请求标志位 1：发送 0：停止发送
    int8_t save_require;    //保存请求标志位
}NEUQ_Menu_Params;

extern NEUQ_Menu_Params Pragma;

typedef enum{
    ACTION_NONE = -1,
    ACTION_SEND_XA, ACTION_SEND_YA, ACTION_SEND_ZA,
    ACTION_SEND_XG, ACTION_SEND_YG, ACTION_SEND_ZG,
}NEUQ_Menu_Action;

void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);

#endif
