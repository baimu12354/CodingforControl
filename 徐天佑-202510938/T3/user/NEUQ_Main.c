#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Main.h"
#include "serial.h"
#include"pid.h"
#include "math.h"
#include <stdlib.h>
#include "td.h"
#include "eso.h"
TD_conctrol TD1;


void User_Main(void)
{
    beta1=3.0*w0;
    beta2=3.0*w0*w0;
    beta3=w0*w0*w0;
    create_TD(&TD1, 1000.0f);



    NEUQ_Debug_init();

    NEUQ_Delay_init();
    NEUQ_GPIO_init(MOTOR_Dir, GPIO_Mode_Output_PushPull, GPIO_Low);
    // NEUQ_Uart_init(UART_0, UART_BAUD_115200);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 5);
    NEUQ_PWM_init(MOTOR_PWM, 4200, 0);
    NEUQ_Encoder_init(Encoder_TIM3, MOTOR_ENCODER_A, MOTOR_ENCODER_B);
    pid_init(&Speed);
    NEUQ_PIT_init_ms(PIT_CH1, 10);
    NEUQ_LCD200_init();

char text[50];

sprintf(text, "%s","neuq");
NEUQ_LCD200_DisplayString(8,170, text);
// NEUQ_PWM_set_duty( MOTOR_PWM, 10000);
// Speed.output=10000;
    while (1) {


        if (Serial_RxFlag) {
            Serial_update();
           
        }
         Motor_SendFeedback();
        //  NEUQ_Delay_ms(100);
    // sprintf(text, "%d %d %d",Speed.actual,Speed.target,Speed.Kp);
    // NEUQ_LCD200_DisplayString(8,170, text);
    
        

    }
}
