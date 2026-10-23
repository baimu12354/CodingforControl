#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Main.h"
#include "serial.h"
#include "eso.h"


volatile char serial_type[1];
uint8_t RxState;
uint8_t pRxPacket;
volatile uint8_t Serial_RxFlag = 0;
volatile uint8_t Serial_RxPacket[16];

extern volatile char serial_type[1];
void NEUQ_PIT_1_CALLBACK(void)
{
    TD_Run(&TD1, td_target);
    Speed.target = TD_get_R1(&TD1);

    motor_get_speed();
    Motor_Control();
    // eso_update(Speed.target, Speed.actual, &Speed.output);
    

}

void NEUQ_PIT_2_CALLBACK(void) {}

 void TS_UART_3_RX_CALLBACK(uint8_t dat)
 {
  
        //  if (RxState == 0) {
        //      if (dat == 'p' || dat=='i' || dat=='d' || dat=='t' ||dat=='f' || dat=='a' || dat=='r' ||'b' || 'w' ||'r') { 
        //          RxState = 1;
        //          pRxPacket = 0;
        //          serial_type[0] = dat;
        //      }
        //  }
        //  else if (RxState == 1) {
        //       if (dat == 'p' || dat=='i' || dat=='d' || dat=='t' ||dat=='f' || dat=='a' || dat=='r' || 'b' || 'w' ||'r')
        //       {
        //          RxState = 1;
        //          pRxPacket = 0;
        //          serial_type[0] = dat;
        //       }
        //       else
        //       {
        //      Serial_RxPacket[0] = dat;
        //      Serial_RxFlag = 1;
        //      RxState = 0;
        //       }
        //  } 
 
 }
void TS_UART_0_RX_CALLBACK(uint8_t dat) 
{ 


          if (RxState == 0)
    {
        if (dat == 'p' || dat == 'i' || dat == 'd' ||dat == 't' || dat == 'f' ||dat=='a' || dat=='r' ||dat=='b' ||dat== 'w' )
        {
            RxState = 1;
            pRxPacket = 0;
            serial_type[0] = dat;
        }
    }
    else
    {
        if (dat == '\r' || dat == '\n')
        {
            Serial_RxPacket[pRxPacket] = '\0';
            Serial_RxFlag = 1;
            RxState = 0;
            pRxPacket = 0;
        }
        else if (pRxPacket < 15)
        {
            Serial_RxPacket[pRxPacket++] = dat;
        }
    }
        
}

void TS_UART_1_RX_CALLBACK(uint8_t dat) { (void)dat; }
void TS_UART_2_RX_CALLBACK(uint8_t dat) { (void)dat; }
void NEUQ_EXTI0_Callback(void) {}
void NEUQ_EXTI1_Callback(void) {}
void NEUQ_EXTI2_Callback(void) {}
void NEUQ_EXTI3_Callback(void) {}
void NEUQ_EXTI4_Callback(void) {}
void NEUQ_EXTI5_Callback(void) {}
void NEUQ_EXTI6_Callback(void) {}
void NEUQ_EXTI7_Callback(void) {}
void NEUQ_EXTI8_Callback(void) {}
void NEUQ_EXTI9_Callback(void) {}
void NEUQ_EXTI10_Callback(void) {}
void NEUQ_EXTI11_Callback(void) {}
void NEUQ_EXTI12_Callback(void) {}
void NEUQ_EXTI13_Callback(void) {}
void NEUQ_EXTI14_Callback(void) {}
void NEUQ_EXTI15_Callback(void) {}