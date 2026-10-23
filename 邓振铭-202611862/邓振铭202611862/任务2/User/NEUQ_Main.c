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
// 陀螺仪零偏（上电静止标定得到，单位 deg/s）
static float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;

// 上电时对静止的陀螺仪采样求平均，得到零偏，用于消除静止角度漂移
// 注意：标定期间必须保持传感器静止
static void Gyro_Bias_Calibrate(void){
    const int SAMPLE_N = 1000;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    for(int i = 0; i < SAMPLE_N; i++){
        NEUQ_MPU6050_read_data();
        sum_x += mpu6050_gyro_x;
        sum_y += mpu6050_gyro_y;
        sum_z += mpu6050_gyro_z;
        NEUQ_Delay_ms(1);
    }
    // 原始 counts 求平均后换算成 deg/s（±2000dps 量程：16.4 LSB/(deg/s)）
    gx_bias = (float)sum_x / (float)SAMPLE_N / 16.4f;
    gy_bias = (float)sum_y / (float)SAMPLE_N / 16.4f;
    gz_bias = (float)sum_z / (float)SAMPLE_N / 16.4f;
}

void User_Main(void){
    key_init();
    Filter_Init();              // 先载入默认滤波参数（alpha=0.85）
    Filter_LoadFromFlash();     // 上电只加载一次：有记录则覆盖为保存值，无记录保持默认值

    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();
    NEUQ_LCD200_init();
    menu_init();
    NEUQ_MPU6050_init();
    Gyro_Bias_Calibrate();      // 上电静止标定陀螺仪零偏（标定时保持传感器不动）
    NEUQ_PIT_init_ms(PIT_CH1,10);

    
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t last_cnt = DWT->CYCCNT;
    while(1){
        NEUQ_MPU6050_read_data();   // 后台持续采集，切页/关闭上传都不停
        MPU_Data_Process();         // 后台持续滤波

        // 计算本次循环真实耗时 dt（秒）
        uint32_t now_cnt = DWT->CYCCNT;
        float dt = (float)(now_cnt - last_cnt) / (float)SystemCoreClock;
        last_cnt = now_cnt;

        // 陀螺仪 deg/s -> rad/s，送入 Mahony 四元数解算
        // 陀螺仪用原始值（不经 0.85 重滤波）进 Mahony，避免相位滞后；
        // 高频噪声由积分本身和 Mahony 的加速度计修正消化
        float gx_rad = (gx - gx_bias) * 0.0174533f;
        float gy_rad = (gy - gy_bias) * 0.0174533f;
        float gz_rad = (gz - gz_bias) * 0.0174533f;

        MahonyUpdate(ax_filter, ay_filter, az_filter, gx_rad, gy_rad, gz_rad, dt);

        if(vofa_en){
            float raw_val, filter_val;
            switch (sel_axis)
            {
            case 0: raw_val = ax;   filter_val = ax_filter; break;
            case 1: raw_val = ay;   filter_val = ay_filter; break;
            case 2: raw_val = az;   filter_val = az_filter; break;
            case 3: raw_val = gx;   filter_val = gx_filter; break;
            case 4: raw_val = gy;   filter_val = gy_filter; break;
            case 5: raw_val = gz;   filter_val = gz_filter; break;
            default: raw_val = 0;   filter_val = 0;         break;
            }
            printf("%.3f,%.3f\n", raw_val, filter_val);
        }
        menu_task();
    }
}
