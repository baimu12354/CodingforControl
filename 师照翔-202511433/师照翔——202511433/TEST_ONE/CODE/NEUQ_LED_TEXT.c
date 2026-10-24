#include"NEUQ_lib_include.h"
#include<stdint.h>


uint32_t stratage_run  = 1;   // 当前正在运行方案
uint32_t stratage_req  = 1;   // 请求切换方案
uint8_t is_finish     = 0;   // 一轮2s周期完成标志
uint8_t step          = 0;   // 当前阶段
uint32_t start_tick   = 0;   // 本阶段起始tick
uint8_t key_temp      = 0;


//LED策略函数
void LED_Stratage1(void)
{

    if(step > 1)
    {
        step = 0;
        start_tick = NEUQ_Count_ms();
    }

    switch(step)
    {
        case 0:
            NEUQ_GPIO_write(PA8, GPIO_High);
            if(NEUQ_Count_ms() - start_tick >= 1000)
            {
                step = 1;
                start_tick = NEUQ_Count_ms();
            }
            break;
        case 1:
            NEUQ_GPIO_write(PA8, GPIO_Low);
            if(NEUQ_Count_ms() - start_tick >= 1000)
            {
                step = 0;
                start_tick = NEUQ_Count_ms();
                is_finish = 1;
            }
            break;
    }
}

void LED_Stratage2(void)
{
    if(step > 1)
    {
        step = 0;
        start_tick = NEUQ_Count_ms();
    }
     switch(step)
    {
        case 0:
            NEUQ_GPIO_write(PA8, GPIO_High);
            if(NEUQ_Count_ms() - start_tick >= 200)
            {
                step = 1;
                start_tick = NEUQ_Count_ms();
            }
            break;
        case 1:
            NEUQ_GPIO_write(PA8, GPIO_Low);
            if(NEUQ_Count_ms() - start_tick >= 1800)
            {
                step = 0;
                start_tick = NEUQ_Count_ms();
                is_finish = 1; // 满2s，周期结束
            }
            break;
    }
}

void LED_Stratage3(void)
{
    if(step > 3)
    {
        step = 0;
        start_tick = NEUQ_Count_ms();
    }
    switch(step)
    {
        case 0:
            NEUQ_GPIO_write(PA8, GPIO_High);
            if(NEUQ_Count_ms() - start_tick >= 200)
            {
                step = 1;
                start_tick = NEUQ_Count_ms();
            }
            break;
        case 1:
            NEUQ_GPIO_write(PA8, GPIO_Low);
            if(NEUQ_Count_ms() - start_tick >= 200)
            {
                step = 2;
                start_tick = NEUQ_Count_ms();
            }
            break;
    
        case 2:
            NEUQ_GPIO_write(PA8, GPIO_High);
            if(NEUQ_Count_ms() - start_tick >= 200)
            {
                step = 3;
                start_tick = NEUQ_Count_ms();
            }
            break;
        case 3:
            NEUQ_GPIO_write(PA8, GPIO_Low);
            if(NEUQ_Count_ms() - start_tick >= 1400)
            {
                step = 0;
                start_tick = NEUQ_Count_ms();
                is_finish = 1;
            }
            break;
    }
}


void LED_Stratage4(void)
{
    if(step > 3)
    {
        step = 0;
        start_tick = NEUQ_Count_ms();
    }
    switch(step)
    {
        case 0:
            NEUQ_GPIO_write(PA8, GPIO_High);
            if(NEUQ_Count_ms() - start_tick >= 200)
            {
                step = 1;
                start_tick = NEUQ_Count_ms();
            }
            break;
        case 1:
            NEUQ_GPIO_write(PA8, GPIO_Low);
            if(NEUQ_Count_ms() - start_tick >= 200)
            {
                step = 2;
                start_tick = NEUQ_Count_ms();
            }
            break;
    case 2:
            NEUQ_GPIO_write(PA8, GPIO_High);
            if(NEUQ_Count_ms() - start_tick >= 600)
            {
                step = 3;
                start_tick = NEUQ_Count_ms();
            }
            break;
        case 3:
            NEUQ_GPIO_write(PA8, GPIO_Low);
            if(NEUQ_Count_ms() - start_tick >= 1000)
            {
                step = 0;
                start_tick = NEUQ_Count_ms();
                is_finish = 1;
            }
            break;
 }
}

static uint32_t s_tick_ms = 0;//累计毫秒
static uint32_t s_us_acc  = 0;//不足 1ms 的微秒余数
static uint32_t s_last_us = 0;//上次读到的计数值
uint32_t NEUQ_Count_ms(void)
{
     uint32_t now_us = NEUQ_Get_Timer_Value_us();
    uint32_t diff_us;

    if (now_us >= s_last_us) {//计算两次基数之间的时间差
        diff_us = now_us - s_last_us;
    } else {
        diff_us = (65536 - s_last_us) + now_us;  
    }
    s_last_us = now_us;
    s_us_acc  += diff_us;// 累加微秒
    s_tick_ms += s_us_acc / 1000;// 满 1ms 进位
    s_us_acc  %= 1000;// 保留不足 1ms 的余数
    return s_tick_ms;
}