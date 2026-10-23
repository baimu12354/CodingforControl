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
#include "Imu_State.h"

ImuData g_imu;
uint8_t g_mpu_ok = 0;

void Imu_Sampling_Update(void)
{
    float alpha = (float)g_config.alpha_milli / 1000.0f;
    uint8_t i;

    NEUQ_MPU6050_read_data();
    //读数据函数读出来的是原始数据，通过MPU的原理可以知道
    //读出的数据既是内部的ADC读数，将独处的数据存进全局变量MPU_ACC_x中
    //transition函数是纯换算，只做除法
    g_imu.raw[0] = MPU6050_acc_transition(mpu6050_acc_x);
    g_imu.raw[1] = MPU6050_acc_transition(mpu6050_acc_y);
    g_imu.raw[2] = MPU6050_acc_transition(mpu6050_acc_z);
    g_imu.raw[3] = MPU6050_gyro_transition(mpu6050_gyro_x);
    g_imu.raw[4] = MPU6050_gyro_transition(mpu6050_gyro_y);
    g_imu.raw[5] = MPU6050_gyro_transition(mpu6050_gyro_z);


    //
    //
    //此处的作用是初始化低通滤波的值，因为一阶滤波的公式是递归的，第一次并没有上一次的结果
    //所以initialized的目的是一次性初始值装载，将第一次采样的原始值给滤波赋初始值，避免产生
    //从0开始的一个虚假的爬升瞬态
    if (!g_imu.initialized)
    {
        for (i = 0; i < 6U; i++)
        {
            g_imu.filtered[i] = g_imu.raw[i];
        }
        g_imu.initialized = 1;
    }
    else
    {
        /* 一阶低通：y(k)=y(k-1)+alpha*[x(k)-y(k-1)]。 */
        for (i = 0; i < 6U; i++)
        {
            g_imu.filtered[i] += alpha * (g_imu.raw[i] - g_imu.filtered[i]);
        }
    }
}



//该函数的目的将float拆成四个字节，打包成VOFA+认识的justFloat帧的模式，通过串口发送出去
//这个函数每次发送两个通道加尾帧共12字节为一次记录
//union的作用即是既能吧float当成一个整体徕卡可能又可以拆分成四个独立的字节，即
//union是重解释的作用，可以当成两种不同的个读法
//十六进制的每一位数字代表了四位，每两个数字即代表了一个字节，即2进制每一位数字代表了一位
//字节转换方法（如何将Float变量转换成二进制编码，通常float四字节对应的是32位，具体自己查，然后再转换成
//十六进制编码再通过串口进行发送
//这是 float 正无穷（+∞）的字节表示。+∞ 的二进制是 0x7F800000，在 STM32（小端序）内存里从低到高就是 00 00 80 7F。
//VOFA+ 用它当"一帧结束"的分隔符——上位机串口收到这一串就知道：前面 8 字节是一帧完整数据，可以画出来了
//float在内存里面本身就是四个字节，但是在c语言的无法直接取出必须要通过转化
//计算机内部的通信只能通过01来完成，通信结束之后可以再通过相同的转换机制将
//发送的过程是从低位发送到高位

void Imu_Sampling_Init(void)
{
    g_mpu_ok = (uint8_t)(NEUQ_MPU6050_init() == 0U ? 0U : 1U);
}
