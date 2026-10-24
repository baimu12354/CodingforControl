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
#include "../code/menu.h"
#include "../code/config.h"
#include "../code/upload.h"
#include "../code/MadgwickAHRS.h"

// **************************** 代码区域 ****************************
void User_Main(void){
  
    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();
    NEUQ_LCD200_init();               // 初始化屏幕（必须先于任何显示）
    NEUQ_MPU6050_init();              // 先初始化传感器，校准才能读到数据

    NEUQ_LCD200_DisplayString(10, 10, "Calibrating...Do not move");

    // ===== 开机静置校准：陀螺仪零偏（500次×5ms≈2.5s，期间保持模块不动）=====
    {
        float sum[3] = {0.0f, 0.0f, 0.0f};
        const int CAL_N = 500;
        for (int i = 0; i < CAL_N; i++) {
            NEUQ_MPU6050_read_data();          // 读一次六轴
            sum[0] += mpu6050_gyro_x;          // 累加角速度原始值
            sum[1] += mpu6050_gyro_y;
            sum[2] += mpu6050_gyro_z;
            NEUQ_Delay_ms(5);                  // 每拍约5ms
        }
        // 平均原始值 -> 转成 °/s 保存（MPU6050_gyro_transition 输出单位就是 °/s）
        g_gyro_bias[0] = MPU6050_gyro_transition((int16_t)(sum[0] / CAL_N));
        g_gyro_bias[1] = MPU6050_gyro_transition((int16_t)(sum[1] / CAL_N));
        g_gyro_bias[2] = MPU6050_gyro_transition((int16_t)(sum[2] / CAL_N));
    }

    NEUQ_LCD200_DisplayString(10, 10, "Calibration Complete!");

    Menu_Init();
    NEUQ_Flash_init();
    Config_Load();                  //开机读取参数
    Madgwick_Init(200.0f);          //姿态解算：200Hz(与PIT_CH1的5ms一致)
    Madgwick_SetBeta(0.1f);         //beta收敛系数
    Key_Init();
    NEUQ_PIT_init_ms(PIT_CH1, 5);   //读取陀螺仪数据，返回原始和滤波值
    NEUQ_PIT_init_ms(PIT_CH2, 10);  //定时提问是否需要发送到vofa
    while(1){
        KeyEvent_t ev = Key_Scan();
        if(ev != KEY_NONE){
            Menu_OnEvent(ev);
            Menu_Render();
        }
        Menu_RefreshDisplay();       //内部封装了 相当于两个开关的if
        if(g_save_flag){
            Config_Save();
            g_save_flag = 0;
        }
        if(g_upload_tick){
            g_upload_tick = 0;
            VOFA_Upload();           //每次都会被提问，内部封装了要不要发送的开关
        } 
    }
}