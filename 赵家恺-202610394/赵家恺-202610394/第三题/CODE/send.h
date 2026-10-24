#ifndef SEND
#define SEND

#include "NEUQ_lib_include.h"

#define head 0xAA
#define tail 0xBB
#define cmd_on 0x01
#define cmd_off 0x02
#define cmd_set_goal 0x03
#define cmd_set_pid  0x04
//#define len_set_goal 0x05
//#define len_set_pid  0x06
void NEUQ_SEND_PUSH  (uint8_t dat);/*放在中断里的缓冲区*/
void NEUQ_FEED_ANAL (void);
void NEUQ_SEND_INIT (void);
void NEUQ_SEND_ANAL (uint8_t dat);/*解包*/
void NEUQ_SEND_TASK (void);/*执行对接这个pid模块，直接在这里面写入pid,完成上传,100MS中断触发一次，此处仅到fifo写入*/
extern volatile float   p, i, d;
extern volatile int32_t goal;
extern volatile int8_t sendflag;
#define PWM_CH2         TIM8_CH1_PC6 
#define PWM_CH         TIM2_CH3_PA2    
#define DIR_PIN2         PC7 
#define DIR_PIN        PA5               

#define RPM_PER_COUNT  1.0f
void pid_task(void);
#endif