#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "../Library/NEUQ_lib/NEUQ_lib_APP/NEUQ_Filter.h"
#include "NEUQ_Main.h"
#include <math.h>
#include "menu.h"
#include "flash.h"
#include "sendvofa.h"
#include "keyqueue.h"
#include "lcd.h"
uint8_t initok =0;



Task2Saved saved = {0};
volatile AXIS vofa_axis = AXIS_NONE;
float filter_alfa_acc = 0.1f;
float filter_alfa_gyro = 0.8f;
volatile imudata imudata_ = {0};



void User_Main(void)
{

    NEUQ_Debug_init();


    NEUQ_Uart_init(UART_3,UART_BAUD_115200 );
    NEUQ_Delay_init(); 
    NEUQ_MPU6050_init(); 
    initok =  NEUQ_Flash_init(); 
    NEUQ_LCD200_init();
    NEUQ_GPIO_init(KEY_UP, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_DOWN, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_ENTER, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_BACK, GPIO_Mode_Input_PullUp, GPIO_High);
    Menu_Init(); 
    flash_load();
    NEUQ_MPU6050_read_data();
    imudata_.acc_x = mpu6050_acc_x; 
    imudata_.acc_y = mpu6050_acc_y; 
    imudata_.acc_z = mpu6050_acc_z;
    imudata_.gyro_x = mpu6050_gyro_x; 
    imudata_.gyro_y = mpu6050_gyro_y;
    imudata_.gyro_z = mpu6050_gyro_z;
    filter_reset(&imudata_);
    Lcd_Draw();
    NEUQ_PIT_init_ms(PIT_CH1, 10);

    while (1) 
    {
        uint8_t i;

        for ( i = 0; i < 4; ++i) 
        {
            if (key_queue_pop(i)) 
            {
                KeyProcess(i);
                 Lcd_Draw();

            }
        }

        
        if (musave) {
            musave = 0;          
            flash_save();
        }


        if (vofa_axis!= AXIS_NONE && menu_value[vofa_axis+6]==1.0f  ) 
        {

            sendvofa();
        }


        if(lcd_fresh==1){
            lcd_fresh=0;
            Lcd_Draw();
        }

    }

}
// 17.4
// -4.5
// 16.3



