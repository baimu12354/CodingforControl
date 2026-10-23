#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Main.h"
#include <stdlib.h>
#include "eso.h"
float td_target = 0;
// int32_t Serial_Read(uint8_t offset)
// {
//     serial_union_t value;
//     value.Byte[0] = Serial_RxPacket[offset];
//     // value.Byte[1] = Serial_RxPacket[offset + 1];
//     // value.Byte[2] = Serial_RxPacket[offset + 2];
//     // value.Byte[3] = Serial_RxPacket[offset + 3];
//     // return value.IntVal;
//     return value.Byte[0];
// }
float Serial_Read(void)
{
    return atof((char *)&Serial_RxPacket[0]);
}

void Serial_update(void)
{
    switch (serial_type[0]) {
        case 'p':
            Speed.Kp = Serial_Read() ;
        
            break;
        case 'i':
            Speed.Ki = Serial_Read() ;
            break;
        case 'd':
            Speed.Kd = Serial_Read() ;
            break;
        case 't':

            td_target = Serial_Read() ;
            // Speed.target = Serial_Read() ;

            break;
        case 'f':
            k_forward = Serial_Read();

            break;
        case 'a':
            k_forward2 = Serial_Read();

            break;
        case 'r':
            TD1.td.R = Serial_Read();
            break;
        case 'b':
            b0=Serial_Read();
            break;
        case 'w':
            w0=Serial_Read();
            break;
    }
    Serial_RxFlag = 0;
}


// void Motor_SendFeedback(void)
// {
//     uint8_t frame[30];
//     float temp_actual = (float)Speed.actual ;//单位： 圈/s
//     float temp_target = (float)Speed.target ;

    
//     memcpy(&frame[0], &temp_actual, 4);
//     frame[4]='\r';
//     frame[5]='\n';
//     memcpy(&frame[6], &temp_target, 4);
//     frame[10]='\r';
//     frame[11]='\n';
//     // memcpy(&frame[12], &Speed.Kp, 4);
//     // frame[16]='\r';
//     // frame[17]='\n';
//     // memcpy(&frame[18], &Speed.Ki, 4);
//     // frame[22]='\r';
//     // frame[23]='\n';
//     // memcpy(&frame[24], &Speed.Kd, 4);
//     // frame[28]='\r';
//     // frame[29]='\n';
//     NEUQ_Uart_write_data(UART_0, frame, sizeof(frame));
// }

void Motor_SendFeedback(void)
{
    // if(fabsf(Speed.actual - Speed.target)==1)
    // {
    //     v_filter = Speed.target;
    // }
    // else 
    // {
    //     v_filter = Speed.actual;
    // }
    char text[100];

    sprintf(text,"%f,%f,%f,%f,%f\n", Speed.actual,Speed.target,Speed.Kp,Speed.Ki,Speed.Kd);

    NEUQ_Uart_write_data(UART_0,(uint8_t *)text,strlen(text));
}
