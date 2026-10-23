#include "NEUQ_lib_include.h"
#include "NEUQ_KEY.h"   

void NEUQ_Key_Init(Key_type *key,GPIO_Ports_ENUM pin,funt_event Press_event){ 
     key->Port=pin ;
    // key->Last_state=GPIO_High;
    //key->Now_state=GPIO_High;
     key->Press_event=Press_event;

     NEUQ_GPIO_init(pin, GPIO_Mode_Input_PullUp, GPIO_High);  //上拉输入
}
/*void NEUQ_Key_Scan(Key_type *key){
    key->Now_state=NEUQ_GPIO_read(key->Port);
    if(key->Last_state==GPIO_High&&key->Now_state==GPIO_Low){
        if(key->Press_event!=NULL){
            key->Press_event();
        }
    }
    key->Last_state=key->Now_state;

}*/

#define KEY_DEBOUNCE_MS  20U     // 消抖时长（ms）

extern volatile uint32_t g_tick;   // 1ms 系统节拍（在 NEUQ_Interrupts.c 的 PIT 中断中累加）
// 非阻塞消抖扫描：
// 电平必须连续稳定 KEY_DEBOUNCE_MS ms 才确认变化，滤除机械抖动；
// 只在“松开→按下”的下降沿返回一次，长按期间不会重复触发；
// 本函数内部无任何延时/等待，主循环随便调用，不影响 LED 时序。
uint8_t Key_Scan(Key_type *key)
{
    uint8_t raw = (NEUQ_GPIO_read(key->Port) == GPIO_Low) ? 0U : 1U;   // 0=按下, 1=松开，raw是实时电平
    static uint8_t  stable = 1U;     // 消抖后的稳定电平
    static uint32_t change_tick = 0U;     // 稳定电平最近一次确认时刻
    uint8_t event = 0U;

    if (raw == stable)
    {
        change_tick = g_tick;             // 与稳定电平一致，持续刷新计时（消抖）
    }
    else if ((uint32_t)(g_tick - change_tick) >= KEY_DEBOUNCE_MS)
    {
        stable=raw;                // 电平变化且稳定保持 KEY_DEBOUNCE_MS ms：确认新电平
        change_tick = g_tick;
        if (stable == 0U)
        {
            event = 1U;                   // 下降沿：一次有效按键（长按只产生一次）
        }
    }
    return event;
}//因为这是一个只会返回一个值（0或1）的函数,所以在主函数中用if(Key_Scan(&key1))来判断按键是否按下,之后再给他命令。（猜想如果试一下把event=1u改成key->Press_event()，然后直接在主函数中调用Key_Scan(&key1)就可以直接调用Onkeypress了）
//之前注释的没有消抖但是能够直接调用Onkeypress，因为它本身里面就有调用.




