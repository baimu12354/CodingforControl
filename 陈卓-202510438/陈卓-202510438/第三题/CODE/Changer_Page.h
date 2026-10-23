#ifndef _CHANGER_PAGE_H_
#define _CHANGER_PAGE_H_

typedef struct 
{
    uint8_t location;
    uint8_t in[3];//最多三级菜单
    uint8_t point;
    uint8_t PWM_WORK;
    float step;
}PAGE_CONTROL;

extern PAGE_CONTROL PAGE;
extern STATE KEY_STATE;
extern uint8_t g_next;
extern uint8_t g_cur;
extern uint8_t Q2_axis;
extern float Q2_kalman_r;
extern float Q2_kalman_q;

void point_control(STATE *KEY_STATE,PAGE_CONTROL *PAGE);
void MENU(PAGE_CONTROL *PAGE);//主菜单
void LED_PLAN_MENU(PAGE_CONTROL *PAGE);//LED闪烁方案菜单
void MPU6050_MENU(PAGE_CONTROL *PAGE);//六轴数据处理菜单
void SHOW_SCREEN_WORK(PAGE_CONTROL *PAGE);
void Page_Init(PAGE_CONTROL *PAGE);
void SCREEN_SHOW_INIT(void);
void MENU_SET(PAGE_CONTROL *PAGE);

#endif