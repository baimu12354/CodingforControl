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
#include "NEUQ_Filter.h"
#include "NEUQ_Attitude.h"

// **************************** 代码区域 ****************************
void User_Main(void){
    // 初始化默认调试串口（printf 走 UART_0，即 VOFA+ 上位机串口）
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();
    // 定时器13计时（用于屏幕定时刷新）
    NEUQ_Timer_init();
    // 按键 GPIO 初始化
    NEUQ_Key_Init();
    // 10ms 定时器，用于按键扫描消抖
    NEUQ_PIT_init_ms(PIT_CH1, KEY_SCAN_PERIOD_MS);
    // 六轴传感器初始化
    NEUQ_MPU6050_init();
    // 姿态解算初始化：静置校准陀螺仪零偏 + 四元数初始化（此时模块需保持静止）
    NEUQ_Attitude_Init();
    // 板载 W25Q16 Flash 初始化（掉电保存滤波参数）
    NEUQ_Flash_init();

    // 上电从 Flash 恢复六个轴的滤波参数；无有效记录时统一使用默认值
    Menu_Flash_DataTypeDef flash_data = {0};
    if(NEUQ_Flash_read(FLASH_SAVE_ADDRESS, (uint8_t *)&flash_data, sizeof(flash_data)) == NEUQ_FLASH_OK
       && flash_data.magic_flag == MAGIC_FLAG){
        for(uint8_t i = 0; i < 6; i++) params.filter_param[i] = flash_data.filter_param[i];
    }
    else{
        for(uint8_t i = 0; i < 6; i++) params.filter_param[i] = FILTER_PARAM_DEFAULT;
    }

    // 构建菜单、初始化 LCD 并刷新首屏
    NEUQ_Menu_Init();

    while(1){
        NEUQ_MPU6050_read_data();   // 读 MPU6050 原始数据
        NEUQ_Filter_Update();       // 六轴后台滤波 + 姿态解算（切页不停止）
        NEUQ_Menu_Task();           // 按键、屏幕刷新、波形上传
    }
}