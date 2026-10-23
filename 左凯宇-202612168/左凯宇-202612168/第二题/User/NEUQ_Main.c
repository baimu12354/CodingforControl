
/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_Main
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

// **************************** 代码区域 ****************************

volatile uint32_t pit_tick = 0;  // 中断写入，主循环读取。
volatile uint32_t filt_tick = 0;


void User_Main(void){

    uint16_t save_sector=NEUQ_FLASH_LAST_SECTOR_ID;
  
    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();
    //按键初始化
    NEUQ_Key_Init();
    //菜单&屏幕初始化
    NEUQ_Menu_Init();
    //Flash初始化
    NEUQ_Flash_init();
    //GPIO初始化
    NEUQ_GPIO_init(PA8, GPIO_Mode_Output_PushPull, GPIO_Low);
    //TIM6
    NEUQ_PIT_init_ms(PIT_CH1,10);
    //初始化MPU6050传感器
    NEUQ_MPU6050_init();
    //零偏标定
    mpu_solve_zero();
    //初始化uart,tx pa9,rx pa10 
    NEUQ_Uart_init(UART_0,UART_BAUD_115200);
    //初始化滤波
    //定义滤波参数
    Filter ax_f, ay_f, az_f;
    Filter gx_f, gy_f, gz_f;
    float filt_a2 = params.filt_a / 1000.0f;
    float filt_g2 = params.filt_g / 1000.0f;
    NEUQ_Filter_init(&ax_f, filt_a2);
    NEUQ_Filter_init(&ay_f, filt_a2);
    NEUQ_Filter_init(&az_f, filt_a2);
    NEUQ_Filter_init(&gx_f, filt_g2);
    NEUQ_Filter_init(&gy_f, filt_g2);
    NEUQ_Filter_init(&gz_f, filt_g2);

//读取，相关变量在滤波的那个头文件
    FilterSave fits;
    if(NEUQ_Flash_read_sector(save_sector,(uint8_t *)&fits,sizeof(fits)) != NEUQ_FLASH_OK){
        params.filt_a = 200;
        params.filt_g = 100;
    }

    if (fits.correct == CORRECTION_NUM){
        params.filt_a = fits.filtaccel;
        params.filt_g = fits.filtgyro;
    }

//写入
    

    while(1){
        filt_a2 = params.filt_a / 1000.0f;
        filt_g2 = params.filt_g / 1000.0f;
        ax_f.filt = ay_f.filt = az_f.filt = filt_a2;
        gx_f.filt = gy_f.filt = gz_f.filt = filt_g2;
        float ax, ay, az, gx, gy, gz;
        float axf, ayf, azf, gxf, gyf, gzf;
        //MPU6050六轴读数，小数，*1000存储，Menu.c中/1000以实现显示小数

        if ((uint32_t)(filt_tick) >= 1U){

            //积分使用
            uint32_t n = filt_tick;
            filt_tick = 0;
            if (n > 5) n = 5;
            float dt = (float)n * 0.01f;

            //读取，带f换算后，不带是原始
            NEUQ_MPU6050_read_data();
            axf = NEUQ_Filter (&ax_f,(MPU6050_acc_transition(mpu6050_acc_x)));
            ayf = NEUQ_Filter (&ay_f,(MPU6050_acc_transition(mpu6050_acc_y)));
            azf = NEUQ_Filter (&az_f,(MPU6050_acc_transition(mpu6050_acc_z)));
            gxf = NEUQ_Filter (&gx_f,(MPU6050_gyro_transition(mpu6050_gyro_x)));
            gyf = NEUQ_Filter (&gy_f,(MPU6050_gyro_transition(mpu6050_gyro_y)));
            gzf = NEUQ_Filter (&gz_f,(MPU6050_gyro_transition(mpu6050_gyro_z)));

            ax = (MPU6050_acc_transition(mpu6050_acc_x));
            ay = (MPU6050_acc_transition(mpu6050_acc_y));
            az = (MPU6050_acc_transition(mpu6050_acc_z));
            gx = (MPU6050_gyro_transition(mpu6050_gyro_x));
            gy = (MPU6050_gyro_transition(mpu6050_gyro_y));
            gz = (MPU6050_gyro_transition(mpu6050_gyro_z));

            params.ax = (int32_t)(axf * 1000);
            params.ay = (int32_t)(ayf * 1000);
            params.az = (int32_t)(azf * 1000);
            params.gyro_x = (int32_t)(gxf * 1000);
            params.gyro_y = (int32_t)(gyf * 1000);
            params.gyro_z = (int32_t)(gzf * 1000);

            mpu_attitude_update(dt, axf, ayf, azf, gxf, gyf, gzf);
            params.roll  = (int32_t)(att.roll  * 1000);
            params.pitch = (int32_t)(att.pitch * 1000);
            params.yaw   = (int32_t)(att.yaw   * 1000);

            //发送数据
            switch (params.wave_mode){
                case 0:
                    printf ("%.2f,%.2f\n",ax,axf);
                    break;
                case 1:
                    printf ("%.2f,%.2f\n",ay,ayf);
                    break;
                case 2:
                    printf ("%.2f,%.2f\n",az,azf);
                    break;
                case 3:
                    printf ("%.2f,%.2f\n",gx,gxf);
                    break;
                case 4:
                    printf ("%.2f,%.2f\n",gy,gyf);
                    break;
                case 5:
                    printf ("%.2f,%.2f\n",gz,gzf);
                    break;
                case 6:
                default:
                    break;
            }
        }

        if ((uint32_t)(pit_tick) >= 2U){
            NEUQ_Menu_Task();
            pit_tick = 0;
        }


    }
}