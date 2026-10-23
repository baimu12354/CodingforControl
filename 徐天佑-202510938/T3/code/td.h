# ifndef __TD_H
# define __TD_H
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
 
#define timeradio 1000.f
#define sign(x) ((x)>0?1:((x)<0?-1:1))
/*****************************/


 
typedef struct 
{
	float Target; //目标值
	float Lastnot0_target;  // 最后一次非零的目标值
 
	struct 
	{
		float R1;  // 跟踪微分器当前跟踪值
		float R2;  // 跟踪微分器当前跟踪速度
		float V1;  // 跟踪微分器预估的跟踪速度
		float V2;  // 跟踪微分器预估的跟踪加速度
		float R;   // *跟踪微分器最大跟踪速度
	}td;  // 跟踪微分器子结构
 
   struct 
	{
		uint32_t Time_p;  // 上次控制的时刻
		uint32_t Time_n;  // 本次控制的时刻
		float Dtime;      // 两次控制之间的时间
	}Time;
 
 
}TD_conctrol;
 
	
static void getTimeStamp(TD_conctrol * TD); // 记录当前的时间戳
static void TD_cal(TD_conctrol * TD);  // 计算跟踪微分器部分
static void TD_inputStatus(TD_conctrol * TD, float target);  // 输入当前状态
void TD_Run(TD_conctrol * TD, float target);  //输入目标值运行
static void reset_TD(TD_conctrol * TD, float new_R);      	// 重设跟踪微分器参数
static void TD_param_init(TD_conctrol * TD, float R);
float TD_get_R1(TD_conctrol * TD);
float TD_get_R2(TD_conctrol * TD);
void create_TD(TD_conctrol* TD,float R);
 
#endif