#include "NEUQ_lib_include.h"

//0XAA 0X55 LEN DIR DATA ... SUM
//AA 55 01 01 01 
//AA 55 01 02 02 
//AA 55 03 03 0A 00 0E
//AA 55 0D 04 00 00 C0 3F 00 00 10 40 00 00 00 3E 9D

typedef union 
{
    float F;
    uint8_t arr[4];
}un;

un floatToChar;

uint8_t buf[256];
 NEUQ_FIFO_t fifo; 
static uint8_t tempData[24];

uint8_t send_buf[256]; 
NEUQ_FIFO_t send_fifo; 

typedef enum 
{
    CHECK_HEAD_ONE=1,CHECK_HEAD_TWO,CHECK_LEN,CHECK_PUT_DATA,CHECK_SUM
}Check_List;//解包枚举




void Cmd_Control(void)
{
//
    switch (tempData[0])
    {
    case CMD_START:
        NEUQ_LCD200_DisplayString(10,10,"start");
        Motor_Data.run=1;
        break;
    case CMD_PUSE:
        NEUQ_LCD200_DisplayString(10,10,"puse");
        Motor_Data.run=0;
        break;
    case CMD_SET_TARGET:
        NEUQ_LCD200_Display_int32(10,10,(int32_t)((int16_t)tempData[1]|(int16_t)tempData[2]<<8),8);
        Motor_Data.target=((int16_t)tempData[1]|(int16_t)tempData[2]<<8)*RATE;
        break;
    case CMD_SET_PID:
        NEUQ_LCD200_DisplayString(10,10,"pid");
        for(uint8_t i=0;i<4;i++)
        {
            floatToChar.arr[i]=tempData[i+1];
        }
        NEUQ_LCD200_DisplayFloat(10,40,floatToChar.F,8,8);
        Motor_Data.Kp=floatToChar.F;
        for(uint8_t i=0;i<4;i++)
        {
            floatToChar.arr[i]=tempData[5+i];
        }
        NEUQ_LCD200_DisplayFloat(10,70,floatToChar.F,8,8);
        Motor_Data.Ki=floatToChar.F;
        for(uint8_t i=0;i<4;i++)
        {
            floatToChar.arr[i]=tempData[9+i];
        }
        NEUQ_LCD200_DisplayFloat(10,100,floatToChar.F,8,8);
        Motor_Data.Kd=floatToChar.F;

        break;
    default:
        break;
    }
}


void Cmd_Inite(void)
{
    NEUQ_Uart_init(UART_0,UART_BAUD_115200);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0,5);
    NEUQ_FIFO_init(&fifo,buf,sizeof(buf));
    // NEUQ_LCD200_DisplayString(10,90,"cmdinitok");
    NEUQ_FIFO_init(&send_fifo,send_buf,sizeof(send_buf));
}





void Cmd_UnpackData(void)
{
    static Check_List mark=CHECK_HEAD_ONE;
    static uint8_t temp_index=0,len=0,sum=0;
    uint8_t byte;
    if(NEUQ_FIFO_get_used(&fifo)>0)
    {
        // uint8_t used = NEUQ_FIFO_get_used(&fifo);
// NEUQ_LCD200_Display_int32(150, 200, (int32_t)used, 8);
        if(NEUQ_FIFO_read_byte(&fifo, &byte)==NEUQ_FIFO_OK)
        {
            switch(mark)
            {
                case CHECK_HEAD_ONE://读到头1
                if(byte==CMD_HEAD_ONE)
                {
                    temp_index=0;

                    // NEUQ_LCD200_DisplayString(10,30,"head1ok");
                    // NEUQ_LCD200_Display_int32(100,30,(int32_t)byte,8);
                    // NEUQ_Delay_ms(1000);
                    // NEUQ_LCD200_Clear();

                    mark=CHECK_HEAD_TWO;
                    sum=byte;
                }
                break;

                case CHECK_HEAD_TWO://读到头2
                if(byte==CMD_HEAD_TWO)
                {
                    // NEUQ_LCD200_DisplayString(10,50,"head2ok");
                    // NEUQ_Delay_ms(1000);
                    // NEUQ_LCD200_Clear();

                    mark=CHECK_LEN;
                    sum+=byte;
                }
                else
                {
                    mark=CHECK_HEAD_ONE;//失败返回读1
                    sum=0;

                    // NEUQ_LCD200_DisplayString(150,50,"faltok");
                    // NEUQ_Delay_ms(1000);
                    // NEUQ_LCD200_Clear();
                }
                break;

                case CHECK_LEN://数据长度
                len=byte;
                mark=CHECK_PUT_DATA;
                sum+=byte;

                // NEUQ_LCD200_DisplayString(10,70,"readdlen");
                // NEUQ_LCD200_Display_int32(100,70,(int32_t)byte,8);
                // NEUQ_Delay_ms(1000);
                //     NEUQ_LCD200_Clear();

                break;

                case  CHECK_PUT_DATA://读数据（含指令

                // NEUQ_LCD200_DisplayString(10,90,"getindata");
                // NEUQ_LCD200_Display_int32(100,90,(int32_t)byte,8);
                // NEUQ_LCD200_Display_int32(150,90,(int32_t)temp_index,8);
                // NEUQ_Delay_ms(100);
                //     NEUQ_LCD200_Clear();

                tempData[temp_index]=byte;
                temp_index++;
                sum+=byte;
                if (temp_index==len)
                {
                    mark=CHECK_SUM;

                    // NEUQ_LCD200_DisplayString(10,110,"datafinish");
                    // NEUQ_LCD200_Display_int32(100,110,(int32_t)byte,8);
                    // NEUQ_Delay_ms(2000);
                    // NEUQ_LCD200_Clear();

                }
                break;
                
                case CHECK_SUM://校验

                // NEUQ_LCD200_Display_int32(190,130,(int32_t)sum,8);
                // NEUQ_LCD200_Display_int32(100,130,(int32_t)byte,8);

                if(byte==sum){
                    
                    // NEUQ_LCD200_DisplayString(10,180,"turntocontrol");

                    Cmd_Control();
                }
                temp_index=0;
                sum=0;

                // NEUQ_LCD200_DisplayString(10,200,"readfinish");
                // NEUQ_Delay_ms(2000);
                //     NEUQ_LCD200_Clear();

                mark=CHECK_HEAD_ONE;
                break;

                default:
                break;

            }
        }
    }
}


uint8_t send[21]={CMD_HEAD_ONE,CMD_HEAD_TWO,17,CMD_SEND,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void Cmd_SendData(void)
{
printf("%d,%d,%.2f,%.2f,%.2f\n",Motor_Data.actual/RATE,Motor_Data.target/RATE,
    Motor_Data.Kp,Motor_Data.Ki,Motor_Data.Kd);


//     send[4]=(uint8_t)(Motor_Data.actual&0xff);
//     send[5]=(uint8_t)((Motor_Data.actual>>8)&0xff);
//     send[6]=(uint8_t)(Motor_Data.target&0xff);
//     send[7]=(uint8_t)((Motor_Data.target>>8)&0xff);

//     floatToChar.F=Motor_Data.Kp;
//     for(uint8_t i=0 ;i<4;i++)
//     {
//         send[8+i]=floatToChar.arr[0+i];
//     }
//     floatToChar.F=Motor_Data.Ki;
//     for(uint8_t i=0;i<4;i++)
//     {
//         send[12+i]=floatToChar.arr[0+i];
//     }
//     floatToChar.F=Motor_Data.Kd;
//     for(uint8_t i=0;i<4;i++)
//     {
//         send[16+i]=floatToChar.arr[0+i];
//     }
//     for(uint8_t i=0;i<18;i++)
//     {
//         send[20]+=send[i];
//     }
    
// // // for(uint8_t i=0;i<5;i++)
// // // {
// // //     send[5]=send[i];
// // // }
//     NEUQ_FIFO_write(&send_fifo, (const uint8_t *)send,sizeof(send));
// send[20]=0;
}




