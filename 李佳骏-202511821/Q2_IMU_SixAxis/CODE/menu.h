/* menu.h - 菜单接口 */

#ifndef _MENU_H_
#define _MENU_H_

#include "stm32f4xx_hal.h"
#include "imu_mpu6050.h"

typedef enum {
    MENU_PAGE_MAIN = 0,     /* 一级：三个入口 */
    MENU_PAGE_SIXAXIS,      /* 二级：六轴数据显示 */
    MENU_PAGE_WAVE,         /* 二级：波形与滤波设置 */
    MENU_PAGE_ATTITUDE      /* 二级：姿态解算（本工程保留入口） */
} menu_page_t;

/* 波形页的四个条目 */
#define MENU_WAVE_ITEM_AXIS     (0U)
#define MENU_WAVE_ITEM_UPLOAD   (1U)
#define MENU_WAVE_ITEM_ALPHA    (2U)
#define MENU_WAVE_ITEM_SAVE     (3U)
#define MENU_WAVE_ITEM_COUNT    (4U)

void        Menu_Init(void);

/* 处理四个按键的事件；返回 1 表示界面需要重画 */
uint8_t     Menu_HandleKey(void);

menu_page_t Menu_GetPage(void);
uint8_t     Menu_GetCursor(void);

imu_axis_t  Menu_GetAxis(void);
uint8_t     Menu_IsUploadOn(void);

uint8_t     Menu_IsEditing(void);
float       Menu_GetEditValue(void);

/* 刚保存成功后的提示标志，取走即清，界面用它显示一行 "saved to flash" */
uint8_t     Menu_TakeSavedFlag(void);

/* 供任务层查询当前正在编辑或观察的轴 */
imu_axis_t  Menu_GetEditingAxis(void);

#endif /* _MENU_H_ */
