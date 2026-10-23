#include "led.h"
uint8_t count = 0;
void LED_Init(void){
    NEUQ_GPIO_init(PA8,GPIO_Mode_Output_PushPull,GPIO_Low);
  
}

static volatile uint8_t nowflag = 1;
static uint8_t preflag = 1;
void LED_Shine(void){
    
    
    if(count==0)nowflag = preflag;
    switch (nowflag)
    {
    case 1:if(count<100)NEUQ_GPIO_write(PA8,GPIO_High);
    if(count>=100)NEUQ_GPIO_write(PA8,GPIO_Low);
        break;
    case 2:
        if(count<20)NEUQ_GPIO_write(PA8,GPIO_High);
        if(count>=20)NEUQ_GPIO_write(PA8,GPIO_Low);
        break;
    case 3:
        if(count<20)NEUQ_GPIO_write(PA8,GPIO_High);
        if(count>=20&&count<40)NEUQ_GPIO_write(PA8,GPIO_Low);
        if(count>=40&&count<60)NEUQ_GPIO_write(PA8,GPIO_High);
        if(count>=60)NEUQ_GPIO_write(PA8,GPIO_Low);
        break;
    case 4:
        if(count<20)NEUQ_GPIO_write(PA8,GPIO_High);
        if(count>=20&&count<40)NEUQ_GPIO_write(PA8,GPIO_Low);
        if(count>=40&&count<100)NEUQ_GPIO_write(PA8,GPIO_High);
        if(count>=100)NEUQ_GPIO_write(PA8,GPIO_Low);
        break;

    
    
    }
}
void LED_Update(void){
    
    KEY_EVENT_ENUM key = KEY_Get();
    if (key==KEY_PRESS)
    {
        preflag++;
        if (preflag>4)
        {
            preflag = 1;
        }
       
    }
   
   
}
uint8_t Getflag(void){
    return nowflag;
}
uint8_t Getpflag(void){
    return preflag;
}
void Count(void){
    count++;
    count%=200;
}