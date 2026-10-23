#ifndef _CHANGER_SCREEN_H_
#define _CHANGER_SCREEN_H_

extern PAGE_CONTROL PAGE;

extern uint8_t g_next;
extern uint8_t g_cur;



void SCREEN_SHOW_INIT(void);
void MENU_SHOW(void);//主菜单
void LED_PLAN_SHOW(void);//LED闪烁方案四个方案
void POINTER(uint8_t change);//光标
void MPU6050_MENU_SHOW(void);//六轴数据处理菜单
void MPU6050_DATA_SHOW(void);//六轴数据处理菜单
void Changer_MPU6050_Data_UPdate(void);//六轴数据处理菜单
void SCREEN_CLEAR(void);//清屏
void SHOW_SCREEN_WORK(PAGE_CONTROL *PAGE);//显示当前页面
void Q2_SETTING_SHOW(void);//滤波设置页面
void ALPHA_SETTING_R_SHOW(void);//滤波系数设置页面
void ALPHA_SETTING_Q_SHOW(void);//滤波系数设置页面
void Q2_ATTITUDE_SHOW(void);//姿态角分辨率设置页面
void LED_WAIT(uint8_t g_next,uint8_t g_cur);


#endif