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

void User_Main(void){

    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();

    NEUQ_KEY_Init();
    NEUQ_PIT_init_ms(PIT_CH1,10);   // 每10ms扫描一次按键(供菜单使用)

    NEUQ_Menu_Init();               // 初始化菜单显示主页面
    NEUQ_MPU6050_init();            // 初始化MPU6050传感器
    NEUQ_IMU_Attitude_Init();       // 四元数姿态初始化

    // 串口输出标定出的 Z 轴陀螺零偏(°/s), 核对 Yaw 漂移
    {
        char bias_str[24];
        NEUQ_VOFA_fmt_float(bias_str, NEUQ_IMU_GetGyroBiasZ(), 3);
        NEUQ_Uart_write_string(UART_0, (uint8_t*)"gyro_bias_z=");
        NEUQ_Uart_write_string(UART_0, (uint8_t*)bias_str);
        NEUQ_Uart_write_string(UART_0, (uint8_t*)" deg/s\r\n");
    }

    uint8_t refresh_cnt = 0;        // LCD刷新计数(10个采样周期≈100ms刷一次)

    // 一阶低通滤波器状态(六轴)
    float fil_ax=0, fil_ay=0, fil_az=0, fil_gx=0, fil_gy=0, fil_gz=0;

    while(1){
        if(mpu_sample_flag){                 // 每10ms采集一次六轴数据
            mpu_sample_flag = 0;
            NEUQ_MPU6050_read_data();

            // 1. 原始物理量
            float ax = MPU6050_acc_transition(mpu6050_acc_x);   // g
            float ay = MPU6050_acc_transition(mpu6050_acc_y);
            float az = MPU6050_acc_transition(mpu6050_acc_z);
            float gx = MPU6050_gyro_transition(mpu6050_gyro_x); // d/s
            float gy = MPU6050_gyro_transition(mpu6050_gyro_y);
            float gz = MPU6050_gyro_transition(mpu6050_gyro_z);

            // 串口输出 Z 轴残角速度( Yaw 漂移速率 °/s), 每 1s 打印一次
            static uint32_t drift_cnt = 0;
            if (++drift_cnt >= 100) {
                drift_cnt = 0;
                char res_str[24];
                NEUQ_VOFA_fmt_float(res_str, gz - NEUQ_IMU_GetGyroBiasZ(), 3);
                NEUQ_Uart_write_string(UART_0, (uint8_t*)"gz_residual=");
                NEUQ_Uart_write_string(UART_0, (uint8_t*)res_str);
                NEUQ_Uart_write_string(UART_0, (uint8_t*)" deg/s\r\n");
            }

            // 2. 一阶低通滤波
            float alpha_acc = params.wave_alpha_acc / 100.0f;
            float alpha_gyro = params.wave_alpha_gyro / 100.0f;
            float fax = NEUQ_Filter_lowpass(ax, &fil_ax, alpha_acc);
            float fay = NEUQ_Filter_lowpass(ay, &fil_ay, alpha_acc);
            float faz = NEUQ_Filter_lowpass(az, &fil_az, alpha_acc);
            float fgx = NEUQ_Filter_lowpass(gx, &fil_gx, alpha_gyro);
            float fgy = NEUQ_Filter_lowpass(gy, &fil_gy, alpha_gyro);
            float fgz = NEUQ_Filter_lowpass(gz, &fil_gz, alpha_gyro);

            // 3. 刷到菜单显示变量
            params.a_x = (int32_t)(ax*1000.0f);   // g
            params.a_y = (int32_t)(ay*1000.0f);
            params.a_z = (int32_t)(az*1000.0f);
            params.g_x = (int32_t)(gx*10.0f);   // d/s
            params.g_y = (int32_t)(gy*10.0f);
            params.g_z = (int32_t)(gz*10.0f);

            // 4. 若开关打开，向VOFA+发送选中轴的 原始+滤波 波形
            if(params.wave_send){
                float ch[2];
                uint8_t valid = 1;
                switch(params.wave_axis){
                    case 0: ch[0]=ax;  ch[1]=fax; break;  // AX
                    case 1: ch[0]=ay;  ch[1]=fay; break;  // AY
                    case 2: ch[0]=az;  ch[1]=faz; break;  // AZ
                    case 3: ch[0]=gx;  ch[1]=fgx; break;  // GX
                    case 4: ch[0]=gy;  ch[1]=fgy; break;  // GY
                    case 5: ch[0]=gz;  ch[1]=fgz; break;  // GZ
                    default: valid = 0; break;
                }
                if(valid) NEUQ_VOFA_send_frame(ch, 2);
            }

            // 5. 四元数姿态更新(更新周期=采样周期=10ms, 即100Hz)
            NEUQ_IMU_Attitude_Update(ax, ay, az, gx, gy, gz, 0.01f);
            EulerAngle euler;
            NEUQ_IMU_GetEuler(&euler);
            params.roll  = (int32_t)(euler.roll  * 100.0f);   // 放大100倍, 菜单按2位小数显示
            params.pitch = (int32_t)(euler.pitch * 100.0f);
            params.yaw   = (int32_t)(euler.yaw   * 100.0f);

            if(++refresh_cnt >= 10){         // 每100ms刷新一次LCD显示
                refresh_cnt = 0;
                NEUQ_Menu_Refresh();
            }
        }

        NEUQ_Menu_Task();           // 读取按键,更新/切换页面
    }
}
