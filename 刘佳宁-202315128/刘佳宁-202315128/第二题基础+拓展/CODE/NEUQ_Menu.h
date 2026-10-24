#ifndef NEUQ_MENU_H
#define NEUQ_MENU_H

#include <stdio.h>
#include "stdint.h"

// Flash 存储地址：板载 W25Q16 最后一个 4KB 扇区起始地址
#define FLASH_SAVE_ADDRESS 0x1FF000

// Flash 存储结构体：六个轴各自一个滤波参数
typedef struct
{
    int32_t filter_param[6];
    uint16_t magic_flag;
} Menu_Flash_DataTypeDef;

#define MAGIC_FLAG 0xA5A5

// 滤波参数默认值与范围
#define FILTER_PARAM_DEFAULT  30
#define FILTER_PARAM_MIN      0
#define FILTER_PARAM_MAX      100
#define FILTER_PARAM_STEP     1

// 菜单参数结构体，存放需要修改的参数（六个轴各自的滤波参数）
typedef struct{
    int32_t filter_param[6];
} NEUQ_Menu_Params;

// 外部声明参数，在 NEUQ_Menu.c 里定义
extern NEUQ_Menu_Params params;

// 函数声明
void NEUQ_Menu_Init(void);
void NEUQ_Menu_Task(void);

#endif
